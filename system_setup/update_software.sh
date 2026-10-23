#!/usr/bin/env bash
# OmniMow software updater: tells the app about a new GitHub release and installs it on request.
#
# The repository the mower was installed from (host_setup.sh) is compared with the newest release
# of OMNIMOW_SOFTWARE_REPO. A release is made from main on GitHub (see .github/workflows/release.yml).
# The result goes to /opt/omnimow/software_update.json, which app_backend sends to the app. When the
# user taps "Update" in the app, app_backend writes /opt/omnimow/update_request, and
# omnimow-software-update.path runs this script: the release is checked out, the user's changes
# in omnimow.env are carried over, and host_setup.sh rebuilds and restarts what changed.
#
#   update_software.sh --check      look for a newer release (omnimow-software-check.timer)
#   update_software.sh --install    install the newest release
#   update_software.sh --request    handle a request from the app (update_request or check_request)
#
# Private repository: put a GitHub token (read access to the repository) in
# /etc/omnimow/github_token (chmod 600).
set -euo pipefail

ENV_FILE=/etc/omnimow/omnimow.env
if [ -f "$ENV_FILE" ]; then
    set -a
    # shellcheck disable=SC1090
    . "$ENV_FILE"
    set +a
fi
REPO="${OMNIMOW_SOFTWARE_REPO:-Ragsie/OmniMow}"
STATUS_FILE=/opt/omnimow/software_update.json
INSTALL_REQUEST=/opt/omnimow/update_request
CHECK_REQUEST=/opt/omnimow/check_request
TOKEN_FILE=/etc/omnimow/github_token
LOG=/var/log/omnimow-update.log

# The repository host_setup.sh ran from: /etc/omnimow/omnimow.env links to its system_setup folder
REPO_DIR="$(dirname "$(dirname "$(readlink -f "$ENV_FILE")")")"
if [ ! -d "$REPO_DIR/.git" ]; then
    # E.g. installed from a downloaded zip: tell the app why there are no updates
    message="$REPO_DIR is not a git clone - install from a clone (git clone) to get updates from the app"
    python3 -c 'import json, sys, time; json.dump({"state": "error", "available": False, "message": sys.argv[2],
"installed": "unknown", "updated_at": int(time.time())}, open(sys.argv[1], "w"), indent=2)' "$STATUS_FILE" "$message"
    echo "$message"
    exit 1
fi
# git as the owner of the clone (as root, git refuses a repository owned by another user)
OWNER="$(stat -c %U "$REPO_DIR")"
g() { sudo -u "$OWNER" git -C "$REPO_DIR" "$@"; }

# One run at a time: a check must not run while an update installs
exec 9> /run/omnimow-software-update.lock
flock 9

# status STATE MESSAGE: writes the status file for the app, keeping the release fields
status() {
    python3 - "$STATUS_FILE" "$1" "$2" "$(g describe --tags --always 2>/dev/null || echo unknown)" \
              "${release_json:-}" << 'EOF'
import json, os, sys, time
path, state, message, installed, release = sys.argv[1:6]
try:
    with open(path) as f:
        data = json.load(f)
except (OSError, ValueError):
    data = {}
if release:
    r = json.loads(release)
    data.update({"latest": r.get("tag_name", ""), "name": r.get("name") or r.get("tag_name", ""),
                 "notes": r.get("body") or "", "published_at": r.get("published_at", ""),
                 "url": r.get("html_url", "")})
data.update({"state": state, "message": message, "installed": installed,
             "available": state == "available", "updated_at": int(time.time())})
with open(path + ".tmp", "w") as f:
    json.dump(data, f, indent=2)
os.chmod(path + ".tmp", 0o644)
os.replace(path + ".tmp", path)
EOF
    echo "[$1] $2"
}

auth=()
[ -s "$TOKEN_FILE" ] && auth=(-H "Authorization: Bearer $(tr -d '[:space:]' < "$TOKEN_FILE")")

# Newest release (drafts and pre-releases are left out) -> release_json, tag
fetch_release() {
    release_json=$(curl -fsSL --retry 3 "${auth[@]}" -H "Accept: application/vnd.github+json" \
                   "https://api.github.com/repos/$REPO/releases/latest") || {
        release_json=""
        status error "Could not read the latest release of $REPO (no network, no release yet, or a private repository without $TOKEN_FILE)"
        exit 1
    }
    tag=$(python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])' <<< "$release_json")
    # From OMNIMOW_SOFTWARE_REPO itself, which need not be the clone's origin (e.g. a clone of a fork)
    git_auth=()
    if [ -s "$TOKEN_FILE" ]; then
        basic=$(printf 'x-access-token:%s' "$(tr -d '[:space:]' < "$TOKEN_FILE")" | base64 -w0)
        git_auth=(-c "http.extraHeader=Authorization: Basic $basic")
    fi
    if ! g "${git_auth[@]}" fetch --quiet --force "https://github.com/$REPO.git" "refs/tags/$tag:refs/tags/$tag"; then
        status error "Could not fetch $tag from $REPO"
        exit 1
    fi
    # A release of another project (no common history) must never be checked out over this one
    if ! g merge-base HEAD "$tag" > /dev/null; then
        status error "$tag of $REPO has no history in common with $REPO_DIR - clone $REPO (git clone https://github.com/$REPO.git) or set OMNIMOW_SOFTWARE_REPO"
        exit 1
    fi
}

check() {
    fetch_release
    # Up to date when the release is part of what is checked out (also on a newer dev checkout)
    if g merge-base --is-ancestor "$tag" HEAD; then
        status up_to_date "The newest version ($tag) is installed"
    else
        status available "Version $tag can be installed"
    fi
}

install() {
    fetch_release
    status installing "Installing $tag - the mower restarts its services when done"
    env_rel=system_setup/omnimow.env
    work=$(mktemp -d)
    trap 'rm -rf "$work"' EXIT
    # omnimow.env holds the user's settings: keep them and merge the release's new settings in
    cp "$REPO_DIR/$env_rel" "$work/mine"
    g show "HEAD:$env_rel" > "$work/base"
    g checkout --quiet -- "$env_rel"
    if ! g checkout --quiet --detach "$tag"; then
        cat "$work/mine" > "$REPO_DIR/$env_rel"
        status failed "Could not check out $tag - local changes in $REPO_DIR? (git status)"
        exit 1
    fi
    g show "HEAD:$env_rel" > "$work/new"
    cp "$work/mine" "$work/merged"
    env_note=""
    if git merge-file -q "$work/merged" "$work/base" "$work/new"; then
        cat "$work/merged" > "$REPO_DIR/$env_rel"
    else
        # Same lines changed by the user and the release: keep the user's file as it was
        cat "$work/mine" > "$REPO_DIR/$env_rel"
        env_note=" - omnimow.env kept as it was, compare it with the release's (git diff $tag)"
    fi
    echo "=== Installing $tag ($(date)) ===" >> "$LOG"
    if OMNIMOW_FORCE_RECREATE=true bash "$REPO_DIR/system_setup/host_setup.sh" --user "$OWNER" >> "$LOG" 2>&1; then
        status up_to_date "Version $tag installed$env_note"
    else
        status failed "Installing $tag failed - see $LOG on the mower$env_note"
        exit 1
    fi
}

case "${1:---check}" in
    --check)   check ;;
    --install) install ;;
    --request)
        if [ -e "$INSTALL_REQUEST" ]; then
            rm -f "$INSTALL_REQUEST" "$CHECK_REQUEST"
            install
        elif [ -e "$CHECK_REQUEST" ]; then
            rm -f "$CHECK_REQUEST"
            check
        fi ;;
    *) echo "Usage: $0 [--check | --install | --request]"; exit 1 ;;
esac

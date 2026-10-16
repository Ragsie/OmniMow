#!/usr/bin/env bash
# OmniMow AI model updater: installs the model attached to the newest GitHub release.
#
# Runs from a systemd timer (omnimow-model-update.timer, set up by host_setup.sh) shortly after
# boot and then every OMNIMOW_MODEL_CHECK_MIN minutes. When the newest release of
# OMNIMOW_MODEL_REPO carries a newer OMNIMOW_MODEL_ASSET than the installed one, it is downloaded
# and swapped in atomically. depth_fusion notices the new file and reloads it without a restart;
# if the new model fails to load, the running one is kept. The replaced model stays as *.previous.
#
#   update_model.sh               check and install a newer model
#   update_model.sh --mark-current   record the newest release as installed without downloading
#                                    (after installing a model by hand with host_setup.sh --model)
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
REPO="${OMNIMOW_MODEL_REPO:-Ragsie/testing}"
ASSET="${OMNIMOW_MODEL_ASSET:-MowerAIn_seg_htp_context.bin}"
MODEL_FILE=/opt/omnimow/models/MowerAIn_seg_htp_context.bin
VERSION_FILE=/opt/omnimow/models/model_version.json
TOKEN_FILE=/etc/omnimow/github_token

auth=()
[ -s "$TOKEN_FILE" ] && auth=(-H "Authorization: Bearer $(tr -d '[:space:]' < "$TOKEN_FILE")")

release=$(curl -fsSL --retry 3 "${auth[@]}" -H "Accept: application/vnd.github+json" \
          "https://api.github.com/repos/$REPO/releases/latest") || {
    echo "Could not read the latest release of $REPO (no network, no release yet, or a private repository without $TOKEN_FILE)"
    exit 1
}

# Tag, asset id/update time (also catches a model replaced within the same release) and API URL
read -r tag asset_key asset_url < <(python3 -c '
import json, sys
release = json.load(sys.stdin)
for a in release.get("assets", []):
    if a["name"] == sys.argv[1]:
        print(release["tag_name"], str(a["id"]) + "-" + a["updated_at"], a["url"])
        break
' "$ASSET" <<< "$release") || true
if [ -z "${tag:-}" ]; then
    echo "The latest release of $REPO has no $ASSET attached"
    exit 1
fi

installed=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("asset", ""))' \
            "$VERSION_FILE" 2>/dev/null || true)

record() {
    python3 -c 'import json,sys; json.dump({"repo": sys.argv[1], "tag": sys.argv[2], "asset": sys.argv[3],
"installed": sys.argv[4]}, open(sys.argv[5], "w"), indent=2)' "$REPO" "$tag" "$asset_key" "$(date -Is)" "$VERSION_FILE"
}

if [ "${1:-}" = "--mark-current" ]; then
    record
    echo "Recorded $REPO $tag as the installed model"
    exit 0
fi

if [ "$installed" = "$asset_key" ] && [ -f "$MODEL_FILE" ]; then
    echo "Model is up to date ($REPO $tag)"
    exit 0
fi

echo "New model: $REPO $tag - downloading $ASSET"
mkdir -p "$(dirname "$MODEL_FILE")"
part="$MODEL_FILE.part"
# The asset's API URL works for public and (with a token) private repositories
curl -fL --retry 3 "${auth[@]}" -H "Accept: application/octet-stream" -o "$part" "$asset_url"
if [ "$(stat -c %s "$part")" -lt 100000 ]; then
    rm -f "$part"
    echo "Downloaded file is too small to be a model - keeping the installed one"
    exit 1
fi
chmod 644 "$part"
[ -f "$MODEL_FILE" ] && cp -p "$MODEL_FILE" "$MODEL_FILE.previous"
mv -f "$part" "$MODEL_FILE"   # Atomic: depth_fusion never reads a half-written model
record
echo "Model $tag installed: $MODEL_FILE (depth_fusion reloads it within seconds)"

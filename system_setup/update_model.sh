#!/usr/bin/env bash
# OmniMow AI model updater: installs the model attached to the newest GitHub release that has one.
#
# Runs from a systemd timer (omnimow-model-update.timer, set up by host_setup.sh) shortly after
# boot and then every OMNIMOW_MODEL_CHECK_MIN minutes. When the newest release of
# OMNIMOW_MODEL_REPO with an OMNIMOW_MODEL_ASSET file carries a newer one than the installed one
# (software releases without a model are skipped), it is downloaded
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

# hardware.env (host_setup.sh) names the model that suits this hardware (.bin NPU / .onnx CPU)
for env_file in /etc/omnimow/omnimow.env /etc/omnimow/hardware.env; do
    if [ -f "$env_file" ]; then
        set -a
        # shellcheck disable=SC1090
        . "$env_file"
        set +a
    fi
done
REPO="${OMNIMOW_MODEL_REPO:-Ragsie/OmniMow}"
ASSET="${OMNIMOW_MODEL_ASSET:-MowerAIn_seg_htp_context.bin}"
MODEL_FILE="/opt/omnimow/models/$ASSET"
VERSION_FILE=/opt/omnimow/models/model_version.json
TOKEN_FILE=/etc/omnimow/github_token

auth=()
[ -s "$TOKEN_FILE" ] && auth=(-H "Authorization: Bearer $(tr -d '[:space:]' < "$TOKEN_FILE")")

releases=$(curl -fsSL --retry 3 "${auth[@]}" -H "Accept: application/vnd.github+json" \
          "https://api.github.com/repos/$REPO/releases?per_page=50") || {
    echo "Could not read the releases of $REPO (no network, or a private repository without $TOKEN_FILE)"
    exit 1
}

# Newest published release with the model: tag, asset id/update time (also catches a model
# replaced within the same release) and API URL. The list is sorted newest first.
read -r tag asset_key asset_url < <(python3 -c '
import json, sys
for release in json.load(sys.stdin):
    if release.get("draft") or release.get("prerelease"):
        continue
    for a in release.get("assets", []):
        if a["name"] == sys.argv[1]:
            print(release["tag_name"], str(a["id"]) + "-" + a["updated_at"], a["url"])
            sys.exit(0)
' "$ASSET" <<< "$releases") || true
if [ -z "${tag:-}" ]; then
    echo "No release of $REPO has $ASSET attached"
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

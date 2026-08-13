#!/usr/bin/env bash
#
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

COMPOSE_FILE="${1:-compose.yml}"
FALLBACK_TAG="latest"

if [[ ! -f "$COMPOSE_FILE" ]]; then
    echo "Error: compose file '$COMPOSE_FILE' not found." >&2
    exit 1
fi

extract_images() {
    yq eval '.services[].image // ""' "$COMPOSE_FILE" | grep -v '^$'
}

if command -v yq >/dev/null 2>&1; then
    mapfile -t IMAGES < <(extract_images)
else
  echo "yq not found, please Go yq: https://github.com/mikefarah/yq."
  exit 1
fi

if [[ ${#IMAGES[@]} -eq 0 ]]; then
    echo "No images found in $COMPOSE_FILE." >&2
    exit 1
fi

echo "Found ${#IMAGES[@]} image(s) in $COMPOSE_FILE:"
printf '  - %s\n' "${IMAGES[@]}"
echo

image_exists_locally() {
    docker image inspect "$1" >/dev/null 2>&1
}

FAILED_IMAGES=()

for image_ref in "${IMAGES[@]}"; do
    if [[ "$image_ref" == *:* ]]; then
        repo="${image_ref%:*}"
        tag="${image_ref##*:}"
        if [[ "$tag" == */* ]]; then
            repo="$image_ref"
            tag="latest"
        fi
    else
        repo="$image_ref"
        tag="latest"
    fi

    if image_exists_locally "${repo}:${tag}"; then
        echo "    Already available locally, skipping pull."
        continue
    fi

    echo "    Not found locally. Pulling ${repo}:${tag} ..."
    if docker pull "${repo}:${tag}"; then
        echo "    OK"
        continue
    fi

    echo "    Not found: ${repo}:${tag}"

    if [[ "$tag" == "$FALLBACK_TAG" ]]; then
        echo "    No fallback available (tag was already '${FALLBACK_TAG}')."
        FAILED_IMAGES+=("${repo}:${tag}")
        continue
    fi

    echo "    Falling back to ${repo}:${FALLBACK_TAG} ..."
    if docker pull "${repo}:${FALLBACK_TAG}"; then
        echo "    Tagging ${repo}:${FALLBACK_TAG} as ${repo}:${tag}"
        docker tag "${repo}:${FALLBACK_TAG}" "${repo}:${tag}"
        echo "    OK (via fallback)"
    else
        echo "    Fallback pull also failed for ${repo}:${FALLBACK_TAG}"
        FAILED_IMAGES+=("${repo}:${tag}")
    fi

    echo
done

echo
if [[ ${#FAILED_IMAGES[@]} -gt 0 ]]; then
    echo "Done, but the following images could not be pulled (even with fallback):"
    printf '  - %s\n' "${FAILED_IMAGES[@]}"
    exit 1
else
    echo "All images pulled successfully."
fi

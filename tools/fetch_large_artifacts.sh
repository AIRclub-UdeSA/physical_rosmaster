#!/usr/bin/env bash
set -euo pipefail

readonly ARTIFACT_URL="https://github.com/AIRclub-UdeSA/physical_rosmaster/releases/download/large-artifacts-v1/physical_rosmaster_large_artifacts_v1.tar.gz"
readonly ARTIFACT_SHA256="ebf52f25958b958c30f1b1999ba0b6e6baa8850fac519202349f3fb8d38dcf05"
readonly ARTIFACT_NAME="physical_rosmaster_large_artifacts_v1.tar.gz"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
download_dir="$(mktemp -d)"
archive_path="${download_dir}/${ARTIFACT_NAME}"

cleanup() {
  rm -f "${archive_path}"
  rmdir "${download_dir}" 2>/dev/null || true
}
trap cleanup EXIT

download() {
  if command -v curl >/dev/null 2>&1; then
    curl -L "${ARTIFACT_URL}" -o "${archive_path}"
  elif command -v wget >/dev/null 2>&1; then
    wget -O "${archive_path}" "${ARTIFACT_URL}"
  else
    echo "ERROR: curl or wget is required to download large artifacts." >&2
    return 1
  fi
}

verify() {
  echo "${ARTIFACT_SHA256}  ${archive_path}" | sha256sum -c -
}

extract() {
  tar -xzf "${archive_path}" -C "${repo_root}"
}

echo "Downloading physical ROSMASTER large artifacts..."
download
echo "Verifying archive checksum..."
verify
echo "Extracting into ${repo_root}..."
extract
echo "Done."

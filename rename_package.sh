#!/bin/bash

HELP_MESSAGE="Usage: $0 NEW_PACKAGE_NAME [--dry-run]

Rename a package by replacing all text occurrences of template_component_package
with the new package name and renaming all matching files and directories.

The required positional argument is the old package name to replace.

Use --dry-run to prevent any filesystem changes while testing the usage.

This script replaces all text occurrences of template_component_package with
NEW_PACKAGE_NAME in all files in the following search paths:
  - ./.devcontainer.json
  - ./.github/workflows/build-test.yml
  - ./aica-package.toml
  - ./source/**

It also replaces all hyphenated occurrences of the package names in the same search paths
(i.e. template-component-package would be replaced with NEW-PACKAGE-NAME).

Finally, it renames all files and directories that contain template_component_package in
their names to the equivalent name using NEW_PACKAGE_NAME in the following search paths:
  - .source/**

Options:
  -n, --dry-run            Echo the new version but do not
                           write changes to any files.

  -h, --help               Show this help message.
"


DRY_RUN=false

POSITIONAL_ARGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    -n | --dry-run) DRY_RUN=true; shift 1;;
    -h | --help) echo "${HELP_MESSAGE}"; exit 0;;

    -*) echo "Unknown option: $1" >&2; echo "${HELP_MESSAGE}"; exit 1;;
    *) POSITIONAL_ARGS+=("$1"); shift 1;;
  esac
done

if [[ "${#POSITIONAL_ARGS[@]}" -ne 1 ]]; then
  echo "${HELP_MESSAGE}"
  exit 1
fi

OLD_NAME=template_component_package
NEW_NAME="${POSITIONAL_ARGS[0]}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"

function rename_file() {
  FILEPATH="${1}"
  BASENAME="$(basename ${FILEPATH})"
  NEW_BASENAME="${BASENAME//$OLD_NAME/$NEW_NAME}"
  NEW_FILEPATH="$(dirname ${FILEPATH})/${NEW_BASENAME}"

  echo "  from: ${FILEPATH}"
  echo "    to: ${NEW_FILEPATH}"

  if [ ${DRY_RUN} == false ]; then
    mv "${FILEPATH}" "${NEW_FILEPATH}"
  fi
}

function replace_text_in_file() {
  FILEPATH="${1}"

  echo "Replacing text in file: $FILEPATH"
  if [ ${DRY_RUN} != false ]; then
    return
  fi

  SED_STR="s/${OLD_NAME}/${NEW_NAME}/g"
  SED_STR_HYPHENATED="${SED_STR//_/-}"

  if [[ "$OSTYPE" == "darwin"* ]]; then
    sed -i '' "${SED_STR}" "${FILEPATH}"
    sed -i '' "${SED_STR_HYPHENATED}" "${FILEPATH}"
  else
    sed -i "${SED_STR}" "${FILEPATH}"
    sed -i "${SED_STR_HYPHENATED}" "${FILEPATH}"
  fi
}

if [ ${DRY_RUN} != false ]; then
  echo "=== THIS IS A DRY RUN! NO FILE OR FILESYSTEM CHANGES WILL BE MADE ==="
  echo
fi

echo "All text occurences of:"
echo "  - ${OLD_NAME}"
echo "  - ${OLD_NAME//_/-}"
echo "will be replaced with:"
echo "  - ${NEW_NAME}"
echo "  - ${NEW_NAME//_/-}"
echo "in the following search paths:"
echo "  - ${SCRIPT_DIR}/.devcontainer.json"
echo "  - ${SCRIPT_DIR}/.github/workflows/build-test.yml"
echo "  - ${SCRIPT_DIR}/aica-package.toml"
echo "  - ${SCRIPT_DIR}/source/**"
echo

replace_text_in_file "${SCRIPT_DIR}/.devcontainer.json"
replace_text_in_file "${SCRIPT_DIR}/.github/workflows/build-test.yml"
replace_text_in_file "${SCRIPT_DIR}/aica-package.toml"

RENAME_DIRECTORIES=()
for FIND_PATH in $(find "${SCRIPT_DIR}/source"); do

  BASENAME=$(basename $FIND_PATH)

  if [ -d $FIND_PATH ]; then
    if [[ $BASENAME == *"$OLD_NAME"* ]]; then
      RENAME_DIRECTORIES+=($FIND_PATH)
    fi
  else
    replace_text_in_file $FIND_PATH
    if [[ $BASENAME == *"$OLD_NAME"* ]]; then
      echo "Renaming file:"
      rename_file $FIND_PATH
    fi
  fi

done

# rename directories in reverse order (from deep to shallow nesting)
for ((i=${#RENAME_DIRECTORIES[@]}-1; i>=0; i--)); do
  echo "Renaming directory:"
  RENAME_DIRECTORY="${RENAME_DIRECTORIES[$i]}"
  rename_file $RENAME_DIRECTORY
done

if [ ${DRY_RUN} == true ]; then
  echo
  echo "=== THIS WAS A DRY RUN! NO FILE OR FILESYSTEM CHANGES WERE MADE ==="
fi

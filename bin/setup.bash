#!/usr/bin/env bash
export_with_directory() {
  local var_name=$1  # Variable name
  local dir_name=$2  # Directory name

  # Check if the variable is already set and not null
  if [ -n "${!var_name}" ]; then
    # If the variable is set and not null, append the directory to it
    export "$var_name=$dir_name${!var_name:+:${!var_name}}"
  else
    # If the variable is not set or null, set it to the directory
    export "$var_name=$dir_name"
  fi
}

export_with_directory PATH ${PAC_WS}/bin

#!/usr/bin/env bash

_px4_completions()
{
    local cur prev cmds sys_names
    COMPREPLY=()

    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    cmds="help create xrce sim robots bash delete"

    if [[ ${COMP_CWORD} -eq 1 ]]; then
        COMPREPLY=( $(compgen -W "${cmds}" -- "${cur}") )
        return 0
    fi

    case "${COMP_WORDS[1]}" in
        create)
            create_opts="--no-gpu --ros-domain-id --mount-dir"
            COMPREPLY=( $(compgen -W "${create_opts}" -- "${cur}") )
            return 0
            ;;
        sim)
            sim_opts="--headless --world"
            COMPREPLY=( $(compgen -W "${sim_opts}" -- "${cur}") )
            return 0
            ;;
        *)
            ;;
    esac

    return 0
}

_pac_completions()
{
    local cur prev cmds sys_names
    COMPREPLY=()

    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    cmds="help create build mission rviz lpac lpac_l1 offboard px4_sim update_world fake_robots bag \
          list logs delete restart restart-px4 bash cmd gps batt journal update \
          pose vel vlp vgps origin pac rqt"

    if [[ ${COMP_CWORD} -eq 1 ]]; then
        COMPREPLY=( $(compgen -W "${cmds}" -- "${cur}") )
        return 0
    fi

    if [[ "${COMP_WORDS[1]}" == "px4_sim" ]]; then
        local i new_words old_cword
        new_words=( px4 )
        for (( i = 2; i <= COMP_CWORD; i++ )); do
            new_words+=( "${COMP_WORDS[i]}" )
        done

        old_cword=$COMP_CWORD
        COMP_WORDS=( "${new_words[@]}" )
        # Since we removed exactly one token ("pac"), shift COMP_CWORD by -1:
        COMP_CWORD=$(( old_cword - 1 ))

        # Now delegate to px4 completion:
        _px4_completions
        return 0
    fi
    case "${COMP_WORDS[1]}" in
        bag|list|logs|delete|restart|restart-px4|bash|gps|batt|journal|update|pose|vel|vlp|vgps)
            if command -v docker >/dev/null 2>&1; then
                sys_names=$(docker ps --format '{{.Names}}' 2>/dev/null)
                COMPREPLY=( $(compgen -W "${sys_names}" -- "${cur}") )
            fi
            return 0
            ;;
        cmd)
            if [[ ${COMP_CWORD} -eq 2 ]]; then
                if command -v docker >/dev/null 2>&1; then
                    sys_names=$(docker ps --format '{{.Names}}' 2>/dev/null)
                    COMPREPLY=( $(compgen -W "${sys_names}" -- "${cur}") )
                fi
                return 0
            else
                COMPREPLY=( $(compgen -c -- "${cur}") )
                return 0
            fi
            ;;
        *)
            ;;
    esac

    return 0
}

complete -F _pac_completions pac
complete -F _px4_completions px4

export_with_directory() {
  local var_name=$1
  local dir_name=$2

  if [ -n "${!var_name}" ]; then
    export "$var_name=$dir_name${!var_name:+:${!var_name}}"
  else
    export "$var_name=$dir_name"
  fi
}

# Validate PAC_WS before using it
if [[ -n "${PAC_WS:-}" && -d "${PAC_WS}" ]]; then
    export_with_directory PATH "${PAC_WS}/bin"
else
    echo "Warning: PAC_WS is not set or does not point to a valid directory" >&2
fi

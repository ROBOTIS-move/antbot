#!/bin/bash
# Copyright (c) 2026, ROBOTIS CO., LTD.
# Authors: SungHyeon Oh

sudo apt update -y
sudo apt remove -y brltty
sudo apt install -y netplan.io minicom

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repos_file="$script_dir/additional_repos.repos"
src_path="$(cd "$script_dir/.." && pwd)"

if [ ! -f "$repos_file" ]; then
    echo "[Error] additional_repos.repos not found: $repos_file"
    return 1
fi

repo_name=""
repo_url=""
repo_version=""

while IFS= read -r line || [ -n "$line" ]; do
    if [[ "$line" =~ ^[[:space:]]{2}([^[:space:]]+):$ ]]; then
        repo_name="${BASH_REMATCH[1]}"
        repo_url=""
        repo_version=""
    elif [[ "$line" =~ url:[[:space:]]*(.+) ]]; then
        repo_url="${BASH_REMATCH[1]}"
    elif [[ "$line" =~ version:[[:space:]]*(.+) ]]; then
        repo_version="${BASH_REMATCH[1]}"
    fi

    if [ -n "$repo_name" ] && [ -n "$repo_url" ] && [ -n "$repo_version" ]; then
        if [ -d "$src_path/$repo_name" ]; then
            echo "[Skip] $repo_name already exists"
        else
            echo "[Clone] $repo_name -b $repo_version"
            git clone -b "$repo_version" "$repo_url" "$src_path/$repo_name"
        fi
        repo_name=""
        repo_url=""
        repo_version=""
    fi
done < "$repos_file"

rosdep update
rosdep install --from-paths "$src_path" --ignore-src -r -y

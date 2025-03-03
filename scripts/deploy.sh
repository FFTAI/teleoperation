#!/bin/bash

rsync -avzhP -e "ssh -o StrictHostKeyChecking=no" --delete \
--exclude='.git/' \
--exclude='__pycache__/' \
--exclude='.venv/' \
--exclude='__pycache__/**' \
--exclude='.venv/**' \
--exclude='.pixi/' \
--exclude='.pixi/**' \
--exclude='data/' \
--exclude='data/**' \
--exclude='outputs/' \
--exclude='outputs/**' \
--exclude='docker/' \
--exclude='docker/**' \
--include='*/' \
./ farts@farts-012.local:~/teleoperation/

#!/usr/bin/env bash

robot=ubuntu@192.168.55.1

exec ssh -t "$robot" \
  'cd ~/l4b3_ws && exec zsh -c "source install/setup.zsh && exec zsh -i"'

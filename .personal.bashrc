alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'
bind 'set bell-style none'

alias cb='colcon build --symlink-install && source install/setup.bash'
alias cbr='rm -r install build log && colcon build --symlink-install && source install/setup.bash'

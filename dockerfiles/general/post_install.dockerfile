# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

#Set user and workdir:
RUN echo "Disk Space:" && df -h / 
RUN chown -R $UNAME /home/$UNAME
USER $UNAME
WORKDIR /home/$UNAME

#Set entrypoint to bash:
ENTRYPOINT ["/bin/bash"]
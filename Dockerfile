# SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0
#
# SPDX-FileCopyrightText: 2024 University of Zaragoza
# SPDX-License-Identifier: AGPL-3.0-or-later
#
# This file is part of AVOCADO, a derivative work of the RVO2 Library.
# Portions of this file are licensed under the Apache License, Version 2.0,
# and modifications are licensed under the GNU Affero General Public License,
# version 3 or later.
#
# If you use AVOCADO in academic work, please cite:
# Martinez-Baselga, D., Sebastián, E., Montijano, E., Riazuelo, L., Sagüés, C., & Montano, L. (2024). AVOCADO: Adaptive Optimal Collision Avoidance driven by Opinion. arXiv preprint arXiv:2407.00507.
# 
# For details, see the LICENSE file at the root of the repository.
# 
# Contact: diegomartinez@unizar.es
# 			esebastian@unizar.es
# 
#

# Use Ubuntu 20.04 as the base image
FROM ubuntu:20.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies for pyenv and Python builds
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    curl \
    git \
    libssl-dev \
    zlib1g-dev \
    libbz2-dev \
    libreadline-dev \
    libsqlite3-dev \
    wget \
    llvm \
    libncurses5-dev \
    libncursesw5-dev \
    xz-utils \
    tk-dev \
    libffi-dev \
    liblzma-dev \
    python3-openssl \
    ca-certificates \
    cmake && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install pyenv
RUN curl https://pyenv.run | bash

# Set up environment variables for pyenv
ENV PATH="/root/.pyenv/bin:/root/.pyenv/shims:/root/.pyenv/versions:$PATH"
RUN echo 'export PATH="/root/.pyenv/bin:$PATH"' >> ~/.bashrc
#RUN echo 'eval "$(pyenv init --path)"' >> ~/.bashrc
RUN echo 'eval "$(pyenv init -)"' >> ~/.bashrc
#RUN echo 'eval "$(pyenv virtualenv-init -)"' >> ~/.bashrc

# Install desired Python version and create a virtual environment
RUN pyenv install 3.8.15
RUN pyenv virtualenv 3.8.15 AVOCADO

# Set the pyenv virtual environment to activate automatically in .bashrc
RUN echo 'pyenv activate AVOCADO' >> ~/.bashrc
RUN echo 'cd /workspace' >> ~/.bashrc

# Set the virtual environment as the default
RUN pyenv global 3.8.15
RUN pyenv global AVOCADO

# Set the working directory inside the container
WORKDIR /workspace

# Copy the requirements file into the container
COPY requirements.txt .

# Install Python dependencies into the virtual environment
RUN /root/.pyenv/versions/AVOCADO/bin/pip install --upgrade pip
RUN /root/.pyenv/versions/AVOCADO/bin/pip install -r requirements.txt

# Default command
CMD ["/bin/bash"]


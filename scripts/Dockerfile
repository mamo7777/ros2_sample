FROM osrf/ros:jazzy-desktop


RUN apt-get update && apt-get install -y \
    tmux \
    vim \
    x11-apps \
    build-essential \
    clang-format \
    libxcb-xinerama0 \
    git \
    vim-nox \
    python3-pip \
    pipx \
    curl

RUN curl -fLo ~/.vim/autoload/plug.vim --create-dirs \
    https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_sample/meitec_ros2_sample/install/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_sample

RUN echo "call plug#begin('~/.vim/plugged')" >> ~/.vimrc && \
    echo "Plug 'rhysd/vim-clang-format'" >> ~/.vimrc && \
    echo "call plug#end()" >> ~/.vimrc

RUN vim +PlugInstall +qall || true

RUN pipx install black

RUN echo "alias black='/root/.local/share/pipx/venvs/black/bin/black'" >> /root/.bashrc

ENTRYPOINT ["/root/ros2_sample/scripts/entrypoint.sh"]

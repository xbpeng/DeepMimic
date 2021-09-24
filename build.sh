
#!/bin/bash
mkdir -p $HOME/.deepmimic-docker

nvidia-docker build -f Dockerfile \
		--build-arg USER=$USER \
		--build-arg HOME=$HOME/.deepmimic-docker . ;

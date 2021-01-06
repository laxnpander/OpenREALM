FROM ubuntu:18.04

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive && \
            apt-get install -yq --no-install-recommends git \
            nano ca-certificates cmake g++ gcc \
            lsb-release gnupg2 curl wget make && rm -rf /var/lib/apt/lists/*

RUN cd / && wget https://raw.githubusercontent.com/laxnpander/OpenREALM/master/tools/install_opencv.sh

RUN cd / && sed -i 's/sudo //g' install_opencv.sh && bash install_opencv.sh && cd ~ && rm -rf *

RUN wget https://raw.githubusercontent.com/laxnpander/OpenREALM/master/tools/install_deps.sh

RUN cd / && sed -i 's/sudo //g' install_deps.sh && apt-get update && export DEBIAN_FRONTEND=noninteractive && \  
	bash install_deps.sh && rm -rf /var/lib/apt/lists/*

RUN cd ~ && rm -rf *

RUN cd / && wget https://raw.githubusercontent.com/laxnpander/OpenREALM/master/tools/install_openrealm_lib.sh

RUN bash /install_openrealm_lib.sh

CMD ["/bin/bash"]





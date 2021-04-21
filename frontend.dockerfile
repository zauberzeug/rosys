FROM node:lts

RUN apt update && apt install openssh-server sudo vim less ack-grep rsync wget curl -y
# we configure the ssh server so we can remotely login via vscode for development
RUN sed -i 's/#Port 22/Port 4022/' /etc/ssh/sshd_config
# for now we just want to run as root to simplify ssh login
RUN userdel -r node
USER root

RUN npm install -g @angular/cli

WORKDIR /

COPY ./frontend/package*.json ./
RUN npm install

COPY entrypoint.sh /docker-entrypoint.sh
ENTRYPOINT ["sh", "/docker-entrypoint.sh"]

WORKDIR /app

RUN ln -s ../node_modules ./

COPY authorized_keys /root/.ssh/

EXPOSE 4200 4022

CMD npm start

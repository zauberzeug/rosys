FROM node:lts

RUN apt update && apt install  openssh-server sudo -y

RUN npm install -g @angular/cli

WORKDIR /

COPY package*.json ./
RUN npm install

COPY docker-entrypoint.sh /docker-entrypoint.sh
ENTRYPOINT ["sh", "/docker-entrypoint.sh"]

WORKDIR /app

RUN ln -s ../node_modules ./

RUN apt install vim less ack-grep -y
RUN sed -i 's/#Port 22/Port 4022/' /etc/ssh/sshd_config

EXPOSE 4200 4022

CMD npm start

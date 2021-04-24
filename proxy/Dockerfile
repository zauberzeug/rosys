FROM nginx:1.16.0-alpine

## Remove default Nginx website
RUN rm -rf /usr/share/nginx/html/*

ARG CONFIG=./nginx.conf
COPY $CONFIG /etc/nginx/
COPY ./proxy_params /etc/nginx/

EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]
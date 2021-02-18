import { NgModule } from "@angular/core";
import { BrowserModule } from "@angular/platform-browser";
import { NgxJoystickModule } from "ngx-joystick";

import { AppRoutingModule } from "./app-routing.module";
import { AppComponent } from "./app.component";
import { HomeComponent } from "./home/home.component";
import { SocketIoConfig } from "./socket-io/socket-io.service";
import { SocketIoModule } from "./socket-io/socket-io.module";

const config: SocketIoConfig = {
  url: "",
  options: { path: "/ws/socket.io" },
};

@NgModule({
  declarations: [AppComponent, HomeComponent],
  imports: [
    BrowserModule,
    AppRoutingModule,
    NgxJoystickModule,
    SocketIoModule.forRoot(config),
  ],
  providers: [],
  bootstrap: [AppComponent],
})
export class AppModule {}

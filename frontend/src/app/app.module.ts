import { NgModule } from "@angular/core";
import { BrowserModule } from "@angular/platform-browser";
import { NgxJoystickModule } from "ngx-joystick";

import { AppRoutingModule } from "./app-routing.module";
import { AppComponent } from "./app.component";
import { HomeComponent } from "./home/home.component";
import { SocketIoConfig } from "./socket-io/socket-io.service";
import { SocketIoModule } from "./socket-io/socket-io.module";
import { MapComponent } from "./map/map.component";
import { MapContainerComponent } from "./map/map-container.component";
import { RobotComponent } from "./map/robot.component";
import { CameraComponent } from "./map/camera.component";
import { BackgroundComponent } from "./map/background.component";
import { LightComponent } from "./map/light.component";
import { APP_BASE_HREF } from "@angular/common";

const config: SocketIoConfig = {
  url: "",
  options: { path: location.pathname + "ws/socket.io" },
};

@NgModule({
  declarations: [
    AppComponent,
    HomeComponent,
    MapComponent,
    MapContainerComponent,
    RobotComponent,
    CameraComponent,
    BackgroundComponent,
    LightComponent,
  ],
  imports: [
    BrowserModule,
    AppRoutingModule,
    NgxJoystickModule,
    SocketIoModule.forRoot(config),
  ],
  providers: [{provide: APP_BASE_HREF, useValue: window.location.origin}],
  bootstrap: [AppComponent],
})
export class AppModule {}

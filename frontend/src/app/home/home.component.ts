import { Component } from "@angular/core";
import { JoystickEvent } from "ngx-joystick";
import { WrappedSocket } from "../socket-io/socket-io.service";

@Component({
  selector: "app-home",
  templateUrl: "./home.component.html",
  styleUrls: ["./home.component.scss"],
})
export class HomeComponent {
  constructor(private socket: WrappedSocket) {
    socket.on('odometry_speed', (data: any) => console.log(data));
  }

  onMove(event: JoystickEvent) {
    console.log(event.data.vector.x, event.data.vector.y);
    this.socket.emit('drive_power', {
      left: event.data.vector.y + event.data.vector.x,
      right: event.data.vector.y - event.data.vector.x,
    });
  }

  stop() {
    console.log("stop");
    this.socket.emit('drive_power', {left: 0, right: 0});
  }
}

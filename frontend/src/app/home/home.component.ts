import { Component } from "@angular/core";
import { JoystickEvent } from "ngx-joystick";
import { WrappedSocket } from "../socket-io/socket-io.service";

@Component({
  selector: "app-home",
  templateUrl: "./home.component.html",
  styleUrls: ["./home.component.scss"],
})
export class HomeComponent {
  left: number = 0;
  right: number = 0;

  constructor(private socket: WrappedSocket) {
    socket.on("robot_pose", (data: any) => console.log(data));

    setInterval(() => {
      if (this.left || this.right) this.sendPower();
    }, 50);
  }

  sendPower() {
    console.log("sending:", this.left, this.right);
    this.socket.emit("drive_power", { left: this.left, right: this.right });
  }

  onMove(event: JoystickEvent) {
    this.left = event.data.vector.y + event.data.vector.x;
    this.right = event.data.vector.y - event.data.vector.x;
  }

  stop() {
    this.left = 0;
    this.right = 0;
    this.sendPower();
  }
}

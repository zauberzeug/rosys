import { Component } from "@angular/core";
import { JoystickEvent } from "ngx-joystick";
import { RobotService } from "../robot.service";

@Component({
  selector: "app-home",
  templateUrl: "./home.component.html",
  styleUrls: ["./home.component.scss"],
})
export class HomeComponent {
  constructor(public robotService: RobotService) {}

  onMove(event: JoystickEvent) {
    this.robotService.sendPower(
      event.data.vector.y + event.data.vector.x,
      event.data.vector.y - event.data.vector.x
    );
  }

  stop() {
    this.robotService.sendPower(0, 0);
  }
}

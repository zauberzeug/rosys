import { Component } from "@angular/core";
import { JoystickEvent } from "ngx-joystick";
import { throttle } from "throttle-debounce";
import { RobotService } from "../robot.service";

@Component({
  selector: "app-home",
  templateUrl: "./home.component.html",
  styleUrls: ["./home.component.scss"],
})
export class HomeComponent {
  constructor(public robotService: RobotService) {}

  sendPower = throttle(100, (left: number, right: number) =>
    this.robotService.sendPower(left, right)
  );

  onMove(event: JoystickEvent) {
    this.sendPower(
      event.data.vector.y + event.data.vector.x,
      event.data.vector.y - event.data.vector.x
    );
  }

  stop() {
    this.robotService.sendPower(0, 0);
  }
}

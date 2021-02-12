import { Component } from "@angular/core";
import { JoystickEvent } from "ngx-joystick";

@Component({
  selector: "app-home",
  templateUrl: "./home.component.html",
  styleUrls: ["./home.component.scss"],
})
export class HomeComponent {
  constructor() {}
  onMove(event: JoystickEvent) {
    console.log(event.data.vector.x, event.data.vector.y);
  }
}

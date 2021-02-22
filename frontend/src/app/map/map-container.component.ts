import { Component } from "@angular/core";
import { RobotService } from "../robot.service";

@Component({
  selector: "app-map-container",
  templateUrl: "./map-container.component.html",
  styleUrls: ["./map-container.component.scss"],
})
export class MapContainerComponent {
  constructor(public robotService: RobotService) {}
}

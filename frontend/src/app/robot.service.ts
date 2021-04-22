import { Injectable } from "@angular/core";
import { Pose } from "./home/pose";
import { WrappedSocket } from "./socket-io/socket-io.service";

@Injectable({
  providedIn: "root",
})
export class RobotService {
  pose?: Pose;
  width: number = 1.0;

  constructor(private socket: WrappedSocket) {
    socket.on("world", (data: any) => (this.width = data.robot.width));
    socket.on("robot_pose", (data: any) => (this.pose = Pose.from_dict(data)));
  }

  sendPower(left: number, right: number) {
    this.socket.emit("drive_power", { left: left, right: right });
  }

  sendTask(task: string) {
    this.socket.emit("task", task);
  }
}

import {
  Component,
  Input,
  OnInit,
  OnChanges,
  SimpleChanges,
} from "@angular/core";
import * as THREE from "three";
import { Object3dComponent } from "./object3d.component";

@Component({
  selector: "app-robot",
  template: "",
  providers: [{ provide: Object3dComponent, useExisting: RobotComponent }],
})
export class RobotComponent
  extends Object3dComponent
  implements OnInit, OnChanges {
  @Input() marker: any;
  @Input() yaw!: number;

  mainMesh!: THREE.Mesh;

  ngOnInit() {
    this.object3d = new THREE.Group();
    this.object3d.castShadow = true;
    this.object3d.name = "robot";

    this.mainMesh = new THREE.Mesh(
      new THREE.BoxGeometry(),
      new THREE.MeshPhongMaterial({ transparent: true })
    );
    this.object3d.add(this.mainMesh);

    this.updateRotation();

    super.ngOnInit();
  }

  ngOnChanges(changes: SimpleChanges) {
    for (const key in changes) {
      if (key == "yaw") this.updateRotation();
    }
    super.ngOnChanges(changes);
  }

  updateRotation() {
    if (!this.object3d) return;

    this.object3d.setRotationFromAxisAngle(
      new THREE.Vector3(0, 0, 1),
      this.yaw
    );
  }
}

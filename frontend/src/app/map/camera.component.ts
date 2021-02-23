import { Component, OnInit } from "@angular/core";
import * as THREE from "three";
import { Object3dComponent } from "./object3d.component";

@Component({
  selector: "app-camera",
  template: "",
  providers: [{ provide: Object3dComponent, useExisting: CameraComponent }],
})
export class CameraComponent extends Object3dComponent implements OnInit {
  ngOnInit() {
    this.object3d = new THREE.PerspectiveCamera(70, 1, 1, 1000);
    this.object3d.up = new THREE.Vector3(0, 0, 1);
    super.ngOnInit();
  }

  update_canvas(width: number, height: number) {
    const camera = this.object3d as THREE.PerspectiveCamera;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  }
}

import {
  AfterContentInit,
  Component,
  ContentChild,
  ContentChildren,
  ElementRef,
  NgZone,
  OnDestroy,
  QueryList,
  ViewChild,
} from "@angular/core";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { CameraComponent } from "./camera.component";
import { Object3dComponent } from "./object3d.component";

@Component({
  selector: "app-map",
  templateUrl: "./map.component.html",
  styleUrls: ["./map.component.scss"],
})
export class MapComponent implements AfterContentInit, OnDestroy {
  @ViewChild("canvas", { static: true })
  canvas!: ElementRef<HTMLCanvasElement>;

  @ContentChild(CameraComponent, { static: true })
  camera_component!: CameraComponent;

  @ContentChildren(Object3dComponent)
  object3d_components!: QueryList<Object3dComponent>;

  scene!: THREE.Scene;
  renderer!: THREE.WebGLRenderer;
  camera!: THREE.PerspectiveCamera;
  orbitControls!: OrbitControls;

  frameId?: number;

  constructor(private ngZone: NgZone) {}

  ngAfterContentInit() {
    this.initScene();
    this.animate();
  }

  ngOnDestroy() {
    if (this.frameId != undefined) cancelAnimationFrame(this.frameId);
  }

  initScene() {
    const canvas = this.canvas.nativeElement;
    this.renderer = new THREE.WebGLRenderer({
      canvas: canvas,
      antialias: true,
      alpha: true,
    });
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    this.camera = this.camera_component.object3d as THREE.PerspectiveCamera;
    this.orbitControls = new OrbitControls(this.camera, canvas);
    this.orbitControls.enableKeys = false;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color().setHex(0xe6f2ff);
    this.scene.fog = new THREE.FogExp2(this.scene.background.getHex(), 0.02);

    this.sync();
    this.object3d_components.changes.subscribe(() => this.sync());
  }

  sync() {
    let sceneIds = this.scene.children.map((object3d) => object3d.uuid);
    let queryIds = this.object3d_components.map((comp) => comp.id);
    this.scene.children
      .filter((object3d: THREE.Object3D) => !queryIds.includes(object3d.uuid))
      .forEach((object3d: THREE.Object3D) => this.scene.remove(object3d));
    this.object3d_components
      .filter((comp: Object3dComponent) => !sceneIds.includes(comp.id))
      .forEach((comp: Object3dComponent) => {
        comp.container = this;
        this.scene.add(comp.object3d);
      });
  }

  resync() {
    while (this.scene.children.length > 0)
      this.scene.remove(this.scene.children[0]);
    this.sync();
  }

  animate() {
    this.ngZone.runOutsideAngular(() => {
      if (document.readyState !== "loading") {
        this.render();
      } else {
        window.addEventListener("DOMContentLoaded", () => this.render());
      }
    });
  }

  render() {
    this.frameId = requestAnimationFrame(() =>
      setTimeout(() => this.render(), 1000 / 30)
    );
    this.resizeCanvasToDisplaySize();
    const camera = this.camera_component.object3d as THREE.PerspectiveCamera;
    this.renderer.render(this.scene, camera);
  }

  resizeCanvasToDisplaySize() {
    const canvas = this.renderer.domElement;
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    if (canvas.width !== width || canvas.height !== height) {
      this.renderer.setSize(width, height, false);
      this.camera_component.update_canvas(canvas.width, canvas.height);
    }
  }
}

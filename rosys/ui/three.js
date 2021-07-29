var scene;
var camera;
var orbitControls;
var loader = new THREE.TextureLoader();
var elements = new Map();

Vue.component("three", {
  template: `<canvas v-bind:id="jp_props.id"></div>`,

  mounted() {
    scene = new THREE.Scene();

    const width = this.$props.jp_props.options.width;
    const height = this.$props.jp_props.options.height;

    camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
    camera.up = new THREE.Vector3(0, 0, 1);
    camera.position.set(0, -3, 5);

    scene.add(new THREE.AmbientLight(0xffffff, 0.7));
    const light = new THREE.DirectionalLight(0xffffff, 0.3);
    light.position.set(5, 10, 40);
    scene.add(light);

    const renderer = new THREE.WebGLRenderer({
      antialias: true,
      alpha: true,
      canvas: document.getElementById(this.$props.jp_props.id),
    });
    renderer.setClearColor("#eee");
    renderer.setSize(width, height);

    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(100, 100),
      new THREE.MeshPhongMaterial({ color: "#eee" })
    );
    ground.translateZ(-0.01);
    ground.name = "ground";
    scene.add(ground);

    const grid = new THREE.GridHelper(100, 100);
    grid.material.transparent = true;
    grid.material.opacity = 0.2;
    grid.rotateX(Math.PI / 2);
    scene.add(grid);

    orbitControls = new THREE.OrbitControls(camera, renderer.domElement);

    const render = function () {
      requestAnimationFrame(() => setTimeout(() => render(), 1000 / 20));
      renderer.render(scene, camera);
    };
    render();

    const raycaster = new THREE.Raycaster();
    const click_handler = (mouseEvent) => {
      let x = (mouseEvent.offsetX / renderer.domElement.width) * 2 - 1;
      let y = -(mouseEvent.offsetY / renderer.domElement.height) * 2 + 1;
      raycaster.setFromCamera({ x: x, y: y }, camera);
      const event = {
        event_type: "onClick",
        vue_type: this.$props.jp_props.vue_type,
        id: this.$props.jp_props.id,
        page_id: page_id,
        websocket_id: websocket_id,
        objects: raycaster.intersectObjects(scene.children, true),
        click_type: mouseEvent.type,
      };
      send_to_server(event, "event");
    };
    document.getElementById(this.$props.jp_props.id).onclick = click_handler;
    document.getElementById(this.$props.jp_props.id).ondblclick = click_handler;
  },

  updated() {
    const jp_elements = Object.values(this.$props.jp_props.options.elements);
    const jp_element_ids = Object.keys(this.$props.jp_props.options.elements);

    elements.forEach((element, id) => {
      if (
        !jp_element_ids.includes(id) ||
        this.$props.jp_props.options.elements[id].modified > element.modified
      ) {
        console.log("remove", id);
        scene.remove(element);
        elements.delete(id);
      }
    });

    jp_elements.forEach((jp_element) => {
      if (!elements.has(jp_element.id)) {
        console.log("add", jp_element.id);
        let element;
        if (jp_element.type == "robot") {
          const jp_shape = jp_element.properties;
          const shape = new THREE.Shape();
          shape.autoClose = true;
          shape.moveTo(jp_shape.outline[0][0], jp_shape.outline[0][1]);
          jp_shape.outline.slice(1).forEach((p) => shape.lineTo(p[0], p[1]));
          const settings = { depth: jp_shape.height, bevelEnabled: false };
          const geometry = new THREE.ExtrudeGeometry(shape, settings);
          if (jp_element.id == "simulation")
            geometry.scale(0.999, 0.999, 0.999);
          if (jp_element.id == "detection") {
            element = new THREE.LineSegments(
              new THREE.EdgesGeometry(geometry),
              new THREE.LineBasicMaterial({
                color: "#6E93D6",
                transparent: true,
                opacity: 0.67,
              })
            );
          } else {
            const material = new THREE.MeshPhongMaterial({
              color: "#6E93D6",
              transparent: true,
              opacity: jp_element.id == "simulation" ? 0.33 : 0.67,
            });
            element = new THREE.Mesh(geometry, material);
          }
        } else if (jp_element.type == "path") {
          let points = [];
          jp_element.properties.splines.forEach((spline) => {
            const curve = new THREE.CubicBezierCurve3(
              new THREE.Vector3(spline.start.x, spline.start.y, 0),
              new THREE.Vector3(spline.control1.x, spline.control1.y, 0),
              new THREE.Vector3(spline.control2.x, spline.control2.y, 0),
              new THREE.Vector3(spline.end.x, spline.end.y, 0)
            );
            points.push(...curve.getPoints(10));
          });
          element = new THREE.Line(
            new THREE.BufferGeometry().setFromPoints(points),
            new THREE.LineBasicMaterial({ color: "orange" })
          );
        } else if (jp_element.type == "image") {
          const projection = jp_element.properties.camera.projection;
          const geometry = new THREE.BufferGeometry();
          const nI = projection[0].length;
          const nJ = projection.length;
          const vertices = [];
          const indices = [];
          const uvs = [];
          for (let j = 0; j < nJ; ++j) {
            for (let i = 0; i < nI; ++i) {
              const X = (projection[j][i] || [0, 0])[0];
              const Y = (projection[j][i] || [0, 0])[1];
              vertices.push(X, Y, 0);
              uvs.push(i / (nI - 1), j / (nJ - 1));
            }
          }
          for (let j = 0; j < nJ - 1; ++j) {
            for (let i = 0; i < nI - 1; ++i) {
              if (
                projection[j][i] &&
                projection[j][i + 1] &&
                projection[j + 1][i] &&
                projection[j + 1][i + 1]
              ) {
                const idx00 = i + j * nI;
                const idx10 = i + j * nI + 1;
                const idx01 = i + j * nI + nI;
                const idx11 = i + j * nI + 1 + nI;
                indices.push(idx00, idx10, idx01);
                indices.push(idx10, idx11, idx01);
              }
            }
          }
          geometry.setIndex(new THREE.Uint32BufferAttribute(indices, 1));
          geometry.setAttribute(
            "position",
            new THREE.Float32BufferAttribute(vertices, 3)
          );
          geometry.setAttribute("uv", new THREE.Float32BufferAttribute(uvs, 2));
          geometry.computeVertexNormals();
          geometry.computeFaceNormals();
          const texture = loader.load("imagedata/" + jp_element.id);
          texture.flipY = false;
          texture.minFilter = THREE.LinearFilter;
          const material = new THREE.MeshLambertMaterial({
            map: texture,
            side: THREE.DoubleSide,
          });
          element = new THREE.Mesh(geometry, material);
          const mac = jp_element.properties.camera.mac;
          element.position.z = 0.0001 * parseInt(mac.substring(0, 2), 16);
        } else if (jp_element.type == "carrot") {
          const cone = new THREE.Mesh(
            new THREE.ConeGeometry(0.1, 0.5),
            new THREE.MeshPhongMaterial({ color: "orange" })
          );
          cone.rotateZ(-Math.PI / 2);
          element = new THREE.Group();
          element.add(cone);
        } else if (jp_element.type == "camera") {
          element = new THREE.Group();
          const intrinsics = jp_element.properties.intrinsics;
          const extrinsics = jp_element.properties.extrinsics;
          if (intrinsics && extrinsics?.rotation) {
            const geometry = new THREE.ConeGeometry(Math.sqrt(0.5), 1, 4);
            geometry.rotateX(-Math.PI / 2);
            geometry.rotateZ(-Math.PI / 4);
            geometry.translate(0, 0, 0.5);
            geometry.scale(
              0.001 * intrinsics.size.width,
              0.001 * intrinsics.size.height,
              0.001 * intrinsics.matrix[0][0]
            );
            const material = new THREE.MeshPhongMaterial({
              color: new THREE.Color(...jp_element.properties.color),
              transparent: true,
              opacity: jp_element.id.endsWith("_") ? 0.33 : 0.67,
            });
            const pyramid = new THREE.Mesh(geometry, material);
            element.add(pyramid);
            element.position.set(...extrinsics.translation);
            const R = new THREE.Matrix4().makeBasis(
              new THREE.Vector3(...extrinsics.rotation[0]),
              new THREE.Vector3(...extrinsics.rotation[1]),
              new THREE.Vector3(...extrinsics.rotation[2])
            );
            element.rotation.setFromRotationMatrix(R.transpose());
          }
        } else {
          console.error("Unknown type:", jp_element.type);
          return;
        }
        element.modified = jp_element.modified;
        scene.add(element);
        elements.set(jp_element.id, element);
      }
      if (jp_element.pose) {
        const element = elements.get(jp_element.id);
        element.position.x = jp_element.pose.x;
        element.position.y = jp_element.pose.y;
        const z_axis = new THREE.Vector3(0, 0, 1);
        element.setRotationFromAxisAngle(z_axis, jp_element.pose.yaw);
      }
    });

    if (
      elements.has("prediction") &&
      this.$props.jp_props.options.follow_robot
    ) {
      const position = elements.get("prediction").position;
      const target = new THREE.Vector3(position.x, position.y, 0);
      orbitControls.target = orbitControls.target
        .clone()
        .multiplyScalar(0.9)
        .add(target.multiplyScalar(0.1));
      camera.lookAt(orbitControls.target);
      camera.updateProjectionMatrix();
    }
  },

  props: {
    jp_props: Object,
  },
});

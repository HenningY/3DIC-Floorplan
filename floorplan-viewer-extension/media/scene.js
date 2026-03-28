// @ts-nocheck
/* global THREE */
(function () {
  let renderer;
  let scene;
  let camera;
  let rootGroup;
  let animationId;
  let resizeObserver;
  /** @type {number} half-height of ortho frustum (world units) */
  let lastOrthoFrustum = 200;

  /** Pastel tier fills (lighter than before) */
  const COLORS = [
    0xa8c8fa,
    0xb8ebd4,
    0xffe0b8,
    0xffc8d8,
    0xe0d4f8,
    0xc8f0ec,
  ];

  /** Add mesh + EdgesGeometry outline for every box */
  function addBoxWithEdges(parent, geometry, meshMaterial, edgeColor) {
    const mesh = new THREE.Mesh(geometry, meshMaterial);
    parent.add(mesh);
    const edges = new THREE.EdgesGeometry(geometry);
    parent.add(
      new THREE.LineSegments(
        edges,
        new THREE.LineBasicMaterial({ color: edgeColor })
      )
    );
  }

  function disposeGroup(group) {
    if (!group) return;
    group.traverse((obj) => {
      if (obj.geometry) obj.geometry.dispose();
      if (obj.material) {
        if (Array.isArray(obj.material)) obj.material.forEach((m) => m.dispose());
        else obj.material.dispose();
      }
    });
  }

  function buildScene(payload) {
    const container = document.getElementById("canvas-container");
    if (!container) return;

    if (!scene) {
      initThree();
    }

    const w = container.clientWidth || 800;
    const h = container.clientHeight || 600;

    if (rootGroup) {
      scene.remove(rootGroup);
      disposeGroup(rootGroup);
      rootGroup = null;
    }

    const {
      outlines,
      modules,
      tsvs,
      tsvHalfSize,
      stackPitch,
      dieThickness,
      moduleThickness: moduleThicknessIn,
    } = payload;

    const numDie = outlines.length;
    const maxW = Math.max(...outlines.map((o) => o.width));
    const maxH = Math.max(...outlines.map((o) => o.height));
    const baseSpan = Math.max(maxW, maxH, 1);
    const moduleThickness = Math.max(
      moduleThicknessIn ?? baseSpan * 0.02,
      0.05
    );

    rootGroup = new THREE.Group();

    // Die slabs (semi-transparent) + edges
    for (let t = 0; t < numDie; t++) {
      const o = outlines[t] || outlines[0];
      const y0 = t * stackPitch;
      const geom = new THREE.BoxGeometry(o.width, dieThickness, o.height);
      geom.translate(o.width / 2, y0 + dieThickness / 2, o.height / 2);
      const mat = new THREE.MeshStandardMaterial({
        color: 0x8899aa,
        transparent: true,
        opacity: 0.22,
        metalness: 0.1,
        roughness: 0.85,
      });
      addBoxWithEdges(rootGroup, geom, mat, 0x4a5d70);
    }

    // Modules (flat boxes on each die surface)
    for (const m of modules) {
      const tier = Math.min(Math.max(m.tier, 0), numDie - 1);
      const o = outlines[tier] || outlines[0];
      const yTop = tier * stackPitch + dieThickness;
      const cx = (m.xll + m.xur) * 0.5;
      const cz = (m.yll + m.yur) * 0.5;
      const mw = Math.max(m.xur - m.xll, 0.5);
      const mh = Math.max(m.yur - m.yll, 0.5);
      const geom = new THREE.BoxGeometry(mw, moduleThickness, mh);
      geom.translate(cx, yTop + moduleThickness * 0.5, cz);
      const color = COLORS[tier % COLORS.length];
      const mat = new THREE.MeshStandardMaterial({
        color,
        metalness: 0.08,
        roughness: 0.82,
      });
      addBoxWithEdges(rootGroup, geom, mat, 0x5c6e82);
    }

    // TSV columns between tiers (vertical along Y)
    const th = tsvHalfSize || 1.5;
    for (const t of tsvs) {
      const tb = Math.min(t.tierBelow, numDie - 1);
      const ta = Math.min(t.tierAbove, numDie);
      const yBottom = tb * stackPitch;
      const yTop = ta * stackPitch;
      const yCenter = (yBottom + yTop) * 0.5;
      const yLen = Math.max(yTop - yBottom, dieThickness * 1.5);
      const geom = new THREE.BoxGeometry(th * 2, yLen, th * 2);
      geom.translate(t.x, yCenter, t.y);
      const mat = new THREE.MeshStandardMaterial({
        color: 0xff8a95,
        emissive: 0x331018,
        metalness: 0.25,
        roughness: 0.5,
      });
      addBoxWithEdges(rootGroup, geom, mat, 0xc43d4d);
    }

    scene.add(rootGroup);

    // Center & frame orthographic camera: fixed 45° oblique top view
    const box = new THREE.Box3().setFromObject(rootGroup);
    const center = new THREE.Vector3();
    box.getCenter(center);
    const size = new THREE.Vector3();
    box.getSize(size);
    const maxDim = Math.max(size.x, size.y, size.z, baseSpan) * 0.65;

    const elev = Math.PI / 4;
    const azim = Math.PI / 4;
    const dir = new THREE.Vector3(
      Math.cos(elev) * Math.cos(azim),
      Math.sin(elev),
      Math.cos(elev) * Math.sin(azim)
    ).normalize();

    const dist = maxDim * 3.2;
    camera.position.copy(center.clone().add(dir.multiplyScalar(dist)));
    camera.near = 0.1;
    camera.far = dist * 20;
    camera.lookAt(center);

    const aspect = w / h;
    const fr = maxDim * 1.35;
    lastOrthoFrustum = fr;
    camera.left = (-fr * aspect) / 2;
    camera.right = (fr * aspect) / 2;
    camera.top = fr / 2;
    camera.bottom = -fr / 2;
    camera.updateProjectionMatrix();

    // Lights
    scene.children
      .filter((c) => c.isLight)
      .forEach((l) => scene.remove(l));
    const amb = new THREE.AmbientLight(0xffffff, 0.55);
    const dirL = new THREE.DirectionalLight(0xffffff, 0.85);
    dirL.position.copy(center.clone().add(new THREE.Vector3(1, 2, 1).multiplyScalar(maxDim)));
    scene.add(amb, dirL);
  }

  function initThree() {
    const container = document.getElementById("canvas-container");
    if (!container || typeof THREE === "undefined") return;
    if (renderer) return;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x12161c);

    const w = container.clientWidth || 800;
    const h = container.clientHeight || 600;
    const aspect = w / h;
    const fr = 200;
    camera = new THREE.OrthographicCamera(
      (-fr * aspect) / 2,
      (fr * aspect) / 2,
      fr / 2,
      -fr / 2,
      0.1,
      50000
    );
    camera.position.set(300, 220, 300);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio || 1);
    renderer.setSize(w, h);
    container.innerHTML = "";
    container.appendChild(renderer.domElement);

    function loop() {
      animationId = requestAnimationFrame(loop);
      renderer.render(scene, camera);
    }
    loop();

    resizeObserver = new ResizeObserver(() => {
      if (!renderer || !camera || !container) return;
      const cw = container.clientWidth;
      const ch = container.clientHeight;
      if (cw < 2 || ch < 2) return;
      const asp = cw / ch;
      const fr = lastOrthoFrustum;
      camera.left = (-fr * asp) / 2;
      camera.right = (fr * asp) / 2;
      camera.top = fr / 2;
      camera.bottom = -fr / 2;
      camera.updateProjectionMatrix();
      renderer.setSize(cw, ch);
    });
    resizeObserver.observe(container);
  }

  window.addEventListener("message", (event) => {
    const msg = event.data;
    if (msg.type === "sceneData") {
      buildScene(msg.payload);
    }
  });
})();

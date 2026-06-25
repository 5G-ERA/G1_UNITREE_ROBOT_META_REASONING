# Inspección en vivo del WebView SLAM (chrome://inspect)

Objetivo: leer el `slam.worker` y la lógica de la web app **ya descifrados en memoria** para
ver cómo recibe la nube del LiDAR (qué topic/mensaje del puente JS usa). El blob en disco
está cifrado; en runtime no.

Pre: emulador Android corriendo con la app `com.unitree.b2dog` instalada (ya lo tienes).
`vconsole` en los assets sugiere que el WebView es depurable.

---

## FASE 1 — Leer el código descifrado (NO necesita robot)

Solo queremos ver CÓMO está escrito (qué topic/bridge usa), no datos en vivo todavía.

1. En el emulador, abre la app Unitree → entra en la pantalla de **SLAM** (el mapa 3D).
   Aunque no haya robot y dé error, el WebView carga y **descifra** el worker.
2. En el Mac, abre **Google Chrome** → barra de direcciones: `chrome://inspect/#devices`.
3. Marca **Discover network targets** y **Discover USB devices**. Espera unos segundos:
   debería aparecer el WebView de la app (algo como "WebView in com.unitree.b2dog").
   - Si NO aparece: el WebView no es depurable en este build → salta a "Plan B".
4. Pulsa **inspect** bajo ese WebView. Se abre DevTools.
5. En DevTools:
   - Pestaña **Sources**. En el panel izquierdo (o `Cmd+P`) busca `slam.worker`, `index-*.js`.
     Ahora se ven **legibles** (descifrados).
   - Busca en todas las fuentes con **`Cmd+Alt+F`**: prueba
     `utlidar`, `voxel`, `cloud`, `subscribe`, `topic`, `lidar`, `rt/`, `onMessage`, `postMessage`.
   - En **Console**, mira el puente nativo: escribe `window` y expande; busca objetos tipo
     `Android`, `JsBridge`, `webkit`, `WebViewManager`. Esos métodos son cómo la web pide
     datos al nativo (y ahí va el nombre del topic).
6. Apunta: el **nombre del topic/mensaje** que el worker/página usa para la nube, y el
   método del bridge (ej. `window.xxx.subscribe("...")`).

## FASE 2 — Ver los datos en vivo (necesita robot)

Si en Fase 1 encuentras el topic, podemos confirmarlo con datos reales:

1. Pon el **Mac en la WiFi del robot** (AP `192.168.12.x`) para que el emulador alcance
   `192.168.12.1` (el emulador sale por la red del Mac).
   - Comprueba: `adb shell ping -c2 192.168.12.1`
2. En la app (emulador), conéctate al robot y abre SLAM. Empezará a llegar la nube.
3. En DevTools del WebView:
   - **Console**: intercepta los mensajes al worker para ver topic + formato. Ej., pega en
     consola algo como:
     ```js
     // log de todo lo que entra por el puente / postMessage
     const _pm = Worker.prototype.postMessage;
     // o hookea el onmessage del worker si tienes su referencia
     ```
   - **Network**: filtra por WS/Fetch por si abre algo a `:9991`/`:8081`.
4. Con el topic + formato confirmados, lo replicamos desde el cliente Python WebRTC
   (suscribir ese topic y decodificar la nube → PointCloud2 → slam_toolbox / .pcd).

---

## Plan B — si el WebView no sale en chrome://inspect

El WebView puede no tener `setWebContentsDebuggingEnabled(true)`. Opciones:
- Forzarlo con Frida en runtime:
  ```
  frida -U -n "Unitree Explore" -e 'Java.perform(()=>{const WV=Java.use("android.webkit.WebView"); WV.setWebContentsDebuggingEnabled(true); console.log("WebView debugging ON");})'
  ```
  (ejecútalo antes de abrir la pantalla SLAM; luego reintenta chrome://inspect)
- O hookear con Frida el puente JS (`addJavascriptInterface`) para loguear las llamadas
  topic/datos entre web y nativo.

---

## Estado para retomar
- slam.worker y three.worker están CIFRADOS en disco (magic dede f7b1). Se descifran en
  runtime → por eso hay que inspeccionar en vivo.
- La web app (Vue, assets/dist) NO abre su propio WebRTC; recibe datos por el puente JS del
  nativo (com.unitree.webrtc: WebViewManagerBridge / JsResponseManager).
- Objetivo: el topic/mensaje de la nube que usa el SLAM, para replicarlo desde Python.

package nz.scuttlebutt.tremolavossbol

// import android.R

// import nz.scuttlebutt.tremolavossbol.tssb.ble.BlePeers

import android.Manifest
import android.app.Activity
import android.app.ActivityManager
import android.bluetooth.BluetoothAdapter
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.ActivityInfo
import android.content.pm.PackageManager
import android.net.*
import android.net.wifi.WifiManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.util.Log
import android.view.View
import android.view.Window
import android.webkit.WebView
import android.widget.Toast
import androidx.annotation.RequiresApi
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.localbroadcastmanager.content.LocalBroadcastManager
import com.google.zxing.integration.android.IntentIntegrator
import nz.scuttlebutt.tremolavossbol.crypto.IdStore
import nz.scuttlebutt.tremolavossbol.tssb.ble.BlePeers
import nz.scuttlebutt.tremolavossbol.tssb.*
import nz.scuttlebutt.tremolavossbol.tssb.ble.BluetoothEventListener
import nz.scuttlebutt.tremolavossbol.utils.Constants
import nz.scuttlebutt.tremolavossbol.games.common.GamesHandler
import nz.scuttlebutt.tremolavossbol.tssb.ble.BleForegroundService
import nz.scuttlebutt.tremolavossbol.utils.HelperFunctions.Companion.toHex
import tremolavossbol.R
import java.net.*
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.thread


// import nz.scuttlebutt.tremolavossbol.MainActivity


class MainActivity : Activity() {
    // lateinit var tremolaState: TremolaState
    lateinit var idStore: IdStore
    lateinit var wai: WebAppInterface
    lateinit var gamesHandler: GamesHandler
    lateinit var tinyIO: IO
    var frontend_ready = false
    val tinyNode = Node(this)
    val tinyRepo = Repo(this)
    val tinyDemux = Demux(this)
    val tinyGoset = GOset(this)
    var settings: Settings? = null
    @Volatile var mc_group: InetAddress? = null
    @Volatile var mc_socket: MulticastSocket? = null
    var ble: BlePeers? = null
    var websocket: WebsocketIO? =null
    val ioLock = ReentrantLock()
    var broadcastReceiver: BroadcastReceiver? = null
    var isWifiConnected = false
    var ble_event_listener: BluetoothEventListener? = null

    /**
     * Receives incoming messages from the ForegroundService.
     */
    private val foregroundserviceReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            val message = intent.getByteArrayExtra("message")
            if (message != null) {
                handleIncomingMessage(message)
            }
        }
    }

    private val isLocationPermissionGranted
        get() = ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) ==
                PackageManager.PERMISSION_GRANTED

    /**
     * Handles incoming messages from the ForegroundService.
     */
    private fun handleIncomingMessage(message: ByteArray) {
        Log.d("MainActivity", "Received message: $message")
        // TODO add here possibly log entries and stuff (everything except BLE)
        wai.eval("b2f_new_message('${message.toHex()}')")
    }

    fun sendMessageToForegroundservice(message: ByteArray) {
        Log.d("MainActivity", "Sending message to Foregroundservice: $message")
        val intent = Intent("MESSAGE_FROM_ACTIVITY")
        intent.putExtra("message", message)
        LocalBroadcastManager.getInstance(this).sendBroadcast(intent)
    }
    /*
    var broadcast_socket: DatagramSocket? = null
    var server_socket: ServerSocket? = null
    var udp: UDPbroadcast? = null
    */
    /*
    val networkRequest = NetworkRequest.Builder()
        .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
        .build()
    private var networkCallback: ConnectivityManager.NetworkCallback? = null
    */
    // var wifiManager: WifiManager? = null
    // private var mlock: WifiManager.MulticastLock? = null
    // lateinit var currentPhotoPath: String
    // private var old_ip_addr: Int = 0 // wifiManager?.connectionInfo!!.ipAddress

    @RequiresApi(Build.VERSION_CODES.O)
    override fun onCreate(savedInstanceState: Bundle?) {
        //val app = applicationContext as TinyApplication
        //app.settings = Settings(this)

        settings = Settings(this)
        super.onCreate(savedInstanceState)

        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT)
        requestWindowFeature(Window.FEATURE_NO_TITLE)
        setContentView(R.layout.activity_main)
        // tremolaState = TremolaState(this)
        idStore = IdStore(this)
        Log.d("MainActivity", "Initiated TremolaState and IdStore")

        val webView = findViewById<WebView>(R.id.webView)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.P) {
            webView.setLayerType(
                View.LAYER_TYPE_SOFTWARE,
                null
            ) // disable acceleration, needed for older WebViews
        }
        Log.d("MainActivity", "Initiated WebView")
        gamesHandler = GamesHandler(idStore.identity)
        webView.addJavascriptInterface(gamesHandler, "GameHandler") // TODO check difference to "GamesHandler"

        Log.d("MainActivity", "Initiated GamesHandler")
        wai = WebAppInterface(this, webView, gamesHandler)
        tinyRepo.upgrade_repo()
        tinyIO = IO(this)
        tinyGoset._include_key(idStore.identity.verifyKey) // make sure our local key is in
        tinyRepo.load()
        tinyGoset.adjust_state()
        tinyDemux.arm_dmx(tinyGoset.goset_dmx,  {buf:ByteArray, aux:ByteArray?, _ -> tinyGoset.rx(buf,aux)}, null)
        tinyDemux.arm_dmx(tinyDemux.want_dmx!!, {buf:ByteArray, aux:ByteArray?, sender:String? -> tinyNode.incoming_want_request(buf,aux,sender)})
        tinyDemux.arm_dmx(tinyDemux.chnk_dmx!!, { buf:ByteArray, aux:ByteArray?, _ -> tinyNode.incoming_chunk_request(buf,aux)})

        webView.clearCache(true)

        Log.d("MainActivity", "Initiated WebAppInterface")
        webView.addJavascriptInterface(wai, "Android")
        webView.addJavascriptInterface(gamesHandler, "GamesHandler")
        Log.d("MainActivity", "Added Javascript interfaces")

        webView.settings.javaScriptEnabled = true
        webView.settings.domStorageEnabled = true

        webView.loadUrl("file:///android_asset/web/tremola.html")
        Log.d("MainActivity", "Initiated UI elements")
        // wifiManager = applicationContext.getSystemService(WIFI_SERVICE) as WifiManager
        // mlock = wifiManager?.createMulticastLock("lock")
        // if (!mlock!!.isHeld) mlock!!.acquire()
        // mkSockets()

        Log.d("IDENTITY", "is ${idStore.identity.toRef()} (${idStore.identity.verifyKey})")

        /*val webView = findViewById<WebView>(R.id.webView)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.P) {
            webView.setLayerType(
                View.LAYER_TYPE_SOFTWARE,
                null
            ) // disable acceleration, needed for older WebViews
        }
        gamesHandler = GamesHandler(idStore.identity)
        webView.addJavascriptInterface(gamesHandler, "GameHandler")*/

        /*wai = WebAppInterface(this, webView, gamesHandler)
        // upgrades repo filesystem if necessary
        tinyRepo.upgrade_repo()
        tinyIO = IO(this, wai)
        tinyGoset._include_key(idStore.identity.verifyKey) // make sure our local key is in
        tinyRepo.load()
        tinyGoset.adjust_state()
        tinyDemux.arm_dmx(tinyGoset.goset_dmx,  {buf:ByteArray, aux:ByteArray?, _ -> tinyGoset.rx(buf,aux)}, null)
        tinyDemux.arm_dmx(tinyDemux.want_dmx!!, {buf:ByteArray, aux:ByteArray?, sender:String? -> tinyNode.incoming_want_request(buf,aux,sender)})
        tinyDemux.arm_dmx(tinyDemux.chnk_dmx!!, { buf:ByteArray, aux:ByteArray?, _ -> tinyNode.incoming_chunk_request(buf,aux)})

        webView.clearCache(true)
        /* no image support in tinyTremola
        webView.webViewClient = object : WebViewClient() {
            override fun shouldInterceptRequest(
                view: WebView,
                request: WebResourceRequest
            ): WebResourceResponse? {
                Log.d("load", "request for URI ${request.url}")
                val bName = request.url.toString().substring(LOCAL_URL_PREFIX.length)
                try {
                    val inputStream = tremolaState.blobStore.fetch(bName)
                    val x = WebResourceResponse(
                        "image/jpeg", null,
                        inputStream
                    )
                    return x
                } catch (e: Exception) {
                    Log.d("fetch error", "${e}")
                }
                return null
            }
        }
        */
        // val webStorage = WebStorage.getInstance()
        webView.addJavascriptInterface(wai, "Android")
        webView.addJavascriptInterface(gamesHandler, "GamesHandler")

        webView.settings.javaScriptEnabled = true
        webView.settings.domStorageEnabled = true

        webView.loadUrl("file:///android_asset/web/tremola.html")
        // webSettings?.javaScriptCanOpenWindowsAutomatically = true

        // prepare for connectivity changes:

        if (networkCallback == null) {
            networkCallback = object : ConnectivityManager.NetworkCallback() {
                override fun onLinkPropertiesChanged(nw: Network, prop: LinkProperties) {
                    // Log.d("onLinkPropertiesChanged", "${nw} ${prop}")
                    super.onLinkPropertiesChanged(nw, prop)
                    // mkSockets()
                }
            }
        }
        */

        broadcastReceiver = object : BroadcastReceiver() {
            override fun onReceive(context: Context?, intent: Intent?) {
                val networkInfo: NetworkInfo? =
                    intent!!.getParcelableExtra(WifiManager.EXTRA_NETWORK_INFO)
                if (networkInfo == null)
                    return
                // Toast.makeText(this@MainActivity, "Wifi State Changed! connected=${networkInfo?.detailedState}", Toast.LENGTH_SHORT).show()
                if (networkInfo.detailedState == NetworkInfo.DetailedState.CONNECTED && !isWifiConnected) {
                    isWifiConnected = true
                    Handler().postDelayed({
                        mkSockets()
                        if (websocket == null)
                            websocket = WebsocketIO(this@MainActivity, settings!!.getWebsocketUrl())
                        websocket!!.start()
                        Log.d("main", "msc_sock ${mc_socket.toString()}")
                    }, 1000)
                } else if (networkInfo.detailedState == NetworkInfo.DetailedState.DISCONNECTED && isWifiConnected) {
                    rmSockets()
                    isWifiConnected = false
                    Log.d("main", "msc_sock ${mc_socket.toString()}")
                    if(websocket != null) {
                        websocket!!.stop()
                        websocket = null
                    }
                }
            }
        }
        registerReceiver(broadcastReceiver, IntentFilter(WifiManager.NETWORK_STATE_CHANGED_ACTION))

        // TODO Change back if turns out to be whack in Foreground Service
        //ble_event_listener = BluetoothEventListener(this)
        //registerReceiver(ble_event_listener, IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED))

        // val lck = ReentrantLock()
        /* disable TCP server and UDP advertisements in the tinyTremola version

        udp = UDPbroadcast(this, tremolaState.wai)

        val t0 = thread(isDaemon=true) {
            try {
                udp!!.beacon(tremolaState.idStore.identity.verifyKey, lck, Constants.SSB_IPV4_TCPPORT)
            } catch (e: Exception) {
                Log.d("beacon thread", "died ${e}")
            }
        }

        val t1 = thread(isDaemon=true) {
            try {
                udp!!.listen(lck)
            } catch (e: Exception) {
                Log.d("listen thread", "died ${e}")
            }
        }*/
        /*val t2 = thread(isDaemon=true)  { // accept loop, robust against reassigned server_socket
             while (true) {
                 var socket: Socket?
                 try {
                     socket = server_socket!!.accept()
                 } catch (e: Exception) {
                     sleep(3000)
                     continue
                 }
                 thread() { // one thread per connection
                     val rpcStream = RpcResponder(tremolaState, socket,
                         Constants.SSB_NETWORKIDENTIFIER)
                     rpcStream.defineServices(RpcServices(tremolaState))
                     rpcStream.startStreaming()
                 }
            }
        }*/


        val t3 = thread(isDaemon=true) {
            try {
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                    tinyIO.senderLoop()
                }
            } catch (e: Exception) {
                Log.d("tssb sender thread", "died ${e}")
            }
        }
        val t4 = thread(isDaemon=true) {
            tinyIO.mcReceiverLoop(ioLock)
        }
        val t5 = thread(isDaemon=true) {
            tinyGoset.loop()
        }
        val t6 = thread(isDaemon=true) {
            tinyNode.loop(ioLock)
        }

        /*
        t0.priority = 10
        t1.priority = 10
        t2.priority = 6
        Log.d("Thread priorities", "${t0.priority} ${t1.priority} ${t2.priority}")
        */

        t3.priority = 8
        t4.priority = 10
        t5.priority = 8
        t6.priority = 8
        Log.d("MainActivity", "Finished onCreate()")
    }

    override fun onBackPressed() {
        wai.eval("onBackPressed();")
    }

    fun _onBackPressed() {
        Handler(this.getMainLooper()).post {
            super.onBackPressed()
        }
    }

    // pt 3 in https://betterprogramming.pub/5-android-webview-secrets-you-probably-didnt-know-b23f8a8b5a0c

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        Log.d("activityResult", "request code ${requestCode} ${data}")
        val result = IntentIntegrator.parseActivityResult(requestCode, resultCode, data)
        if (requestCode == 808) {
            if (data == null)
                return
            // val text = data.getStringExtra("text")
            val voice = data.getByteArrayExtra("codec2")
            if (voice != null)
                wai.return_voice(voice) // public_post_with_voice(text, voice)
            return
        }
        if (result != null) {
            Log.d("activityResult", result.toString())
            val cmd = when {
                result.contents == null -> "qr_scan_failure();"
                else -> "qr_scan_success('" + result.contents + "');"
            }
            wai.eval(cmd)
        /* disabled in tinyTremola
        }  else if (requestCode == 1001 && resultCode == RESULT_OK) { // media pick
            val pictureUri = data?.data
            val bitmap = when {
                Build.VERSION.SDK_INT < Build.VERSION_CODES.P /* 28 */ ->
                    MediaStore.Images.Media.getBitmap(this.contentResolver, pictureUri)
                else -> @RequiresApi(Build.VERSION_CODES.P) {
                    val src = ImageDecoder.createSource(this.contentResolver, pictureUri!!)
                    ImageDecoder.decodeBitmap(src)
                }
            }
            val ref = tremolaState.blobStore.storeAsBlob(bitmap)
            tremolaState.wai.eval("b2f_new_image_blob('${ref}')")
        } else if (requestCode == 1002 && resultCode == RESULT_OK) { // camera
            val ref = tremolaState.blobStore.storeAsBlob(currentPhotoPath)
            tremolaState.wai.eval("b2f_new_image_blob('${ref}')")
        */
        /* no file loading in tinyTremola
        } else if (requestCode == 808 && resultCode == RESULT_OK) { // voice
            val voice = "abc" // result!!.contents
            tremolaState.wai.eval("b2f_new_voice('${voice}')")
        */
        } else if (requestCode == 555 && resultCode == RESULT_OK) { // enable fine grained location
            ble?.startBluetooth()
        }
        super.onActivityResult(requestCode, resultCode, data)
    }

    override fun onResume() {
        Log.d("MainActivity", "onResume")
        super.onResume()

        /*
        try {
            (getSystemService(CONNECTIVITY_SERVICE) as ConnectivityManager)
                .registerNetworkCallback(networkRequest, networkCallback!!)
        } catch (e: Exception) {}
        */

        /*try {
            ble = BlePeers(this)
            ble?.startBluetooth()
        } catch (e: Exception) {
            ble = null
        }*/

        Log.d("MainActivity", "Starting websocket soon ...")
        websocket = WebsocketIO(this, settings!!.getWebsocketUrl())
        websocket!!.start()
        Log.d("MainActivity", "Started websocket")

        //val filter = IntentFilter("MESSAGE_FROM_SERVICE")
        //LocalBroadcastManager.getInstance(this).registerReceiver(foregroundserviceReceiver, filter)
        if (isForegroundServiceRunning()) { return }
        if (!settings!!.isBleEnabled()) {
            Toast.makeText(this, "Bluetooth is disabled!", Toast.LENGTH_SHORT).show()
            return
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M && !isLocationPermissionGranted) {
            ActivityCompat.requestPermissions(this, arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION), 555)
            return
        }
        val intent = Intent(this, BleForegroundService::class.java)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            Log.d("MainActivity", "Starting BLE service")
            applicationContext.startForegroundService(intent)  // Für Android 8.0 und höher
        } else {
            Log.d("MainActivity", "Starting BLE service")
            applicationContext.startService(intent)  // Für ältere Android-Versionen
        }
    }

    override fun onPause() {
        Log.d("onPause", "")
        super.onPause()
        ble?.stopBluetooth()

        if (websocket != null)
            websocket!!.stop()

        /*
        try {
            (getSystemService(CONNECTIVITY_SERVICE) as ConnectivityManager)
                .unregisterNetworkCallback(networkCallback!!)
        } catch (e: Exception) {}
        */
    }

    override fun onStop() {
        Log.d("onStop", "")
        super.onStop()
    }
    override fun onDestroy() {
        Log.d("onDestroy", "")
        /*
        try { broadcast_socket?.close() } catch (e: Exception) {}
        broadcast_socket = null
        try { server_socket?.close() } catch (e: Exception) {}
        server_socket = null
        */
        super.onDestroy()
        ble?.stopBluetooth()

        if (websocket != null) {
            websocket!!.stop()
        }

        // TODO change back if turns out to be whack in Foreground Service
        //unregisterReceiver(broadcastReceiver)
        //unregisterReceiver(ble_event_listener)
    }

    fun mkSockets() {
        /* disable UDP advertisements in the tinyTremola version

        try { broadcast_socket?.close() } catch (e: Exception) {}
        broadcast_socket = null
        try {
            broadcast_socket = DatagramSocket(null)
            broadcast_socket?.reuseAddress = true
            broadcast_socket?.broadcast = true // really necessary ?
            val any = InetAddress.getByAddress(ByteArray(4))
            broadcast_socket?.bind(InetSocketAddress(any, Constants.SSB_IPV4_UDPPORT)) // where to listen
        } catch (e: Exception) {
            Log.d("create broadcast socket", "${e}")
            broadcast_socket = null
        }
        Log.d("new bcast sock", "${broadcast_socket}, UDP port ${broadcast_socket?.localPort}")
        */

        if(!settings!!.isUdpMulticastEnabled())
            return

        rmSockets()
        try {
            mc_group = InetAddress.getByName(Constants.SSB_VOSSBOL_MC_ADDR)
            mc_socket= MulticastSocket(Constants.SSB_VOSSBOL_MC_PORT)
            // mc_socket?.reuseAddress = true
            // mc_socket?.broadcast = true // really necessary ?
            // val any = InetAddress.getByAddress(ByteArray(4))
            // mc_socket?.bind(InetSocketAddress(any, Constants.SSB_VOSSBOL_MC_PORT)) // where to listen
            mc_socket?.loopbackMode = true
            mc_socket?.joinGroup(mc_group)
        } catch (e: Exception) {
            Log.d("mkSockets exc", e.toString())
            rmSockets()
        }
        /* disable TCP server in the tinyTremola version

        // val wifiManager = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        if (old_ip_addr == 0 || old_ip_addr != wifiManager?.connectionInfo!!.ipAddress) {
            try {
                server_socket?.close()
            } catch (e: Exception) {
            }
            server_socket = ServerSocket(Constants.SSB_IPV4_TCPPORT)
            old_ip_addr = wifiManager?.connectionInfo!!.ipAddress
            Log.d(
                "SERVER TCP addr is new",
                "${Formatter.formatIpAddress(wifiManager?.connectionInfo!!.ipAddress)}:${server_socket!!.localPort}"
            )
        } else {
            Log.d(
                "SERVER TCP addr did not change:",
                "${Formatter.formatIpAddress(wifiManager?.connectionInfo!!.ipAddress)}:${server_socket!!.localPort}"
            )
        }
        */
    }

    fun rmSockets() {
        try { mc_socket?.leaveGroup(mc_group); mc_socket?.close() } catch (e: Exception) {}
        mc_group = null
        mc_socket = null
    }

    /**
     * Checks if the BLE foreground service is already running.
     */
    private fun isForegroundServiceRunning(): Boolean {
        val manager = getSystemService(Context.ACTIVITY_SERVICE) as ActivityManager
        for (service in manager.getRunningServices(Int.MAX_VALUE)) {
            if (BleForegroundService::class.java.name == service.service.className) {
                Log.d("MainActivity", "Service is already running.")
                return true
            }
        }
        Log.d("MainActivity", "Service was not yet running.")
        return false
    }
}
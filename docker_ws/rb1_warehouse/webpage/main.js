var app = new Vue({
    el: '#app',
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        set: false,
        build_map:false,
        rosbridge_address: '',
        port: '9090',

        // 3D stuff
        viewer: null,
        tfClient: null,
        urdfClient: null,
        a: false,
        mapViewer: null,
        mapGridClient: null,
        interval: null,

        goal: null,

        start_action: false,

        dragging: false,
        x: 'no',
        y: 'no',

        dragCircleStyle: {
            margin: '0px',
            top: '0px',
            left: '0px',
            display: 'none',
            width: '75px',
            height: '75px',
        },
        
        // joystick valules
        joystick: {
            vertical: 0,
            horizontal: 0,
        },

        // publisher
        pubInterval: null,
        send: false,
        send_first_point: false,
        send_second_point: false,

        // action
        action: {
            goal: { position: {x: 0, y: 0, z: 0} },
            feedback: { position: 0, state: 'idle' },
            result: { success: false },
            status: { status: 0, text: '' },
        },
    
        // idk

        AutoLocalizationState: '',
        AutoLocalizationResponse: '',
        AutoLocalizationMsg: '',
        AutoLocalizationLog: 'Inactive',



        ShelfPositionState: true,
        ShelfPositionResponse: '',
        ShelfPositionMsg: '',
        ShelfPositionLog: 'Inactive',

        ApproachState: true,
        ApproachResponse: '',
        ApproachMsg: '',
        ApproachLog: 'Inactive',

        PatrolBehaviorState: true,
        PatrolBehaviorResponse: '',
        PatrolBehaviorMsg: '',
        PatrolBehaviorLog: 'Inactive',

        // Topics
        up_topic: 'elevator_up',
        down_topic: 'elevator_down',
        cmd_vel_topic: '/cmd_vel'
    },
    // helper methods to connect to ROS
    methods: {
        connect: function() {
            this.loading = true

            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })
            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                this.connected = true
                this.loading = false
                this.a = true

                this.pubInterval = setInterval(this.VelocityPub, 100);
                this.PatrolBehaviorLog = 'Active'
                this.ApproachLog = 'Active'
                this.ShelfPositionLog = 'Active'
                this.AutoLocalizationLog =  'Active'


            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
                this.PatrolBehaviorLog = 'Inactive'
                this.ApproachLog = 'Inactive'
                this.ShelfPositionLog = 'Inactive'
                this.AutoLocalizationLog =  'Inactive'

                clearInterval(this.pubInterval)

            })
        },
        elevator_up: function(){

            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: this.up_topic,
                messageType: 'std_msgs/msg/String'
            })

            let message = new ROSLIB.Message({
                data: ''
            })
            let count = 0;
            let intervalId = setInterval(() => {
                topic.publish(message);
                count++;
                if (count === 3) {
                    clearInterval(intervalId);
                }
            }, 1000); // Publica cada 1000 ms (1 segundo)
        },
        elevator_down: function(){

            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: this.down_topic,
                messageType: 'std_msgs/msg/String'
            })

            let message = new ROSLIB.Message({
                data: ''
            })
            let count = 0;
            let intervalId = setInterval(() => {
                topic.publish(message);
                count++;
                if (count === 3) {
                    clearInterval(intervalId);
                }
            }, 1000); // Publica cada 1000 ms (1 segundo)
        },

        requestBehaviorSrv(){


            this.PatrolBehaviorState = false;


            let setBoolClient = new ROSLIB.Service({
                ros: this.ros,
                name: '/patrol_behavior_server',
                serviceType: 'std_srvs/SetBool'
            });

            // Crear una solicitud de servicio con un valor booleano
            let setBoolRequest = new ROSLIB.ServiceRequest({
                data: true
            });
            this.PatrolBehaviorResponse = "Running...";

            let self = this; // Guardar el contexto de `this`

            setBoolClient.callService(setBoolRequest, (result) => {
                console.log('Service call on '
                    + setBoolClient.name
                    + ' completed with result: '
                    + result.success);

                self.PatrolBehaviorResponse = result.success;
                self.PatrolBehaviorMsg = result.message;
                self.PatrolBehaviorState = false;


            });

            // Limpiar el mensaje después de 3 segundos
            setTimeout(() => {
            this.PatrolBehaviorResponse = '';
            }, 6000);
            
            console.log('Se ha enviado la solicitud a Shelf Position Server');

        },
        requestApproachShelfSrv(){

            this.ApproachState = false;


            let setBoolClient = new ROSLIB.Service({
                ros: this.ros,
                name: '/approach_shelf_server',
                serviceType: 'std_srvs/SetBool'
            });

            // Crear una solicitud de servicio con un valor booleano
            let setBoolRequest = new ROSLIB.ServiceRequest({
                data: true
            });

            this.ApproachResponse = "Running...";

            let self = this; // Guardar el contexto de `this`


            setBoolClient.callService(setBoolRequest, function(result) {
                console.log('Service call on '
                    + setBoolClient.name
                    + ' completed with result: '
                    + result.success);


                self.ApproachResponse = result.success;
                self.ApproachMsg = result.message;
                self.ApproachState = false;



            });

            // Limpiar el mensaje después de 3 segundos
            setTimeout(() => {
            this.ApproachResponse = '';
            }, 6000);

            console.log('Se ha enviado la solicitud a Shelf Position Server');

        },
        requestShelfPositionSrv() {


            this.ShelfPositionState = false;

            let setBoolClient = new ROSLIB.Service({
                ros: this.ros,
                name: '/shelf_position_server',
                serviceType: 'std_srvs/SetBool'
            });

            // Crear una solicitud de servicio con un valor booleano
            let setBoolRequest = new ROSLIB.ServiceRequest({
                data: true
            });

            this.ShelfPositionResponse = "Running...";

            let self = this; // Guardar el contexto de `this`


            setBoolClient.callService(setBoolRequest, function(result) {
                console.log('Service call on '
                    + setBoolClient.name
                    + ' completed with result: '
                    + result.success);

                self.ShelfPositionResponse = result.success;
                self.ShelfPositionMsg = result.message;
                self.ShelfPositionState = false;
            });

            // Limpiar el mensaje después de 3 segundos
            setTimeout(() => {
            this.ShelfPositionResponse = '';
            }, 6000);

            console.log('Se ha enviado la solicitud a Shelf Position Server');
        },
        requestAutoLocalizationSrv() {


            this.AutoLocalizationState = false;

            let setBoolClient = new ROSLIB.Service({
                ros: this.ros,
                name: '/auto_localization_server',
                serviceType: 'std_srvs/SetBool'
            });

            // Crear una solicitud de servicio con un valor booleano
            let setBoolRequest = new ROSLIB.ServiceRequest({
                data: true
            });

            this.AutoLocalizationResponse = "Running...";

            let self = this; // Guardar el contexto de `this`

            setBoolClient.callService(setBoolRequest, function(result) {
                console.log('Service call on '
                    + setBoolClient.name
                    + ' completed with result: '
                    + result.success);

                self.AutoLocalizationResponse = result.success;
                self.AutoLocalizationMsg = result.message;
                self.AutoLocalizationState = false;
            });
            // Limpiar el mensaje después de 3 segundos
            setTimeout(() => {
            this.ShelfPositionResponse = '';
            }, 6000);

            console.log('Se ha enviado la solicitud a Shelf Position Server');
        },
        disconnect: function() {
            this.ros.close()
            this.goal = null
        },
        startDrag() {
            this.dragging = true
            this.x = this.y = 0
        },
        stopDrag() {
            this.dragging = false
            this.x = this.y = 'no'
            this.dragCircleStyle.display = 'none'
            this.resetJoystickVals()
        },
        doDrag(event) {

            if (this.dragging) {
                this.x = event.offsetX
                this.y = event.offsetY
                let ref = document.getElementById('dragstartzone')
                this.dragCircleStyle.display = 'inline-block'

                let minTop = ref.offsetTop - parseInt(this.dragCircleStyle.height) / 2
                let maxTop = minTop + 100
                let top = this.y + minTop
                this.dragCircleStyle.top = `${top}px`

                let minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2
                let maxLeft = minLeft + 100
                let left = this.x + minLeft
                this.dragCircleStyle.left = `${left}px`

                this.setJoystickVals()
            }

        },
        setJoystickVals() {
            this.joystick.vertical = -1 * ((this.y / 100) - 0.5)
            this.joystick.horizontal = +1 * ((this.x / 100) - 0.5)
        },
        resetJoystickVals() {
            this.joystick.vertical = 0
            this.joystick.horizontal = 0
        },
        VelocityPub(){
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: this.cmd_vel_topic,
                messageType: 'geometry_msgs/msg/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: this.joystick.vertical, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: -this.joystick.horizontal, },
            })
            topic.publish(message)
            console.log('VelocityPub está funcionando. Mensaje publicado:', message);

        },
        handleServiceServers: function(event){
            event.preventDefault(event); // Previene el comportamiento predeterminado del formulario
            $.ajax(
                {
                url: $('#init-srvs-form').attr('action'), // Usa la accion del formulario como URL
                type: 'POST',
                data: $('#init-srvs-form').serialize(),  // Envía los datos del formulario,
                                                        // incluyendo el CSRF token
                success: function(data) {
                    $('#log').html('<p>Status: ' + data.status + '</p><p>Output: ' + data.output + '</p>');
                },
                error: function(xhr, status, error) {
                    $('#log').html('<p>Error: ' + error + '</p>');
                }
                }
            )

        },
        handleRB1Start: function(event) {
            event.preventDefault(); // Previene el comportamiento predeterminado del formulario

            $.ajax({
                url: $('#start-rb1-form').attr('action'), // Usa la acción del formulario como URL
                type: 'POST',
                data: $('#start-rb1-form').serialize(), // Envía los datos del formulario, incluyendo el CSRF token
                success: function(data) {
                    $('#log').html('<p>Status: ' + data.status + '</p><p>Output: ' + data.output + '</p>');
                },
                error: function(xhr, status, error) {
                    $('#log').html('<p>Error: ' + error + '</p>');
                }
            });
        },
        handleStartNav2: function(event){
            event.preventDefault(); // Previene comportamiento por defecto del formulario
            $.ajax({
                url: $("#start-nav2-form").attr('action'), // Usa la accion del formulario como URL
                type: 'POST',
                data: $('#start-nav2-form').serialize(), // Envía los datos del formulario, incluyendo el CSRF token
                success: function(data) {
                    $('#log').html('<p>Status: ' + data.status + '</p><p>Output: ' + data.output + '</p>');
                },
                error: function(xhr, status, error) {
                    $('#log').html('<p>Error: ' + error + '</p>');
                }
            })
        },

    },
    mounted() {
        window.addEventListener('mouseup', this.stopDrag)
        this.interval = setInterval(() => {
            if (this.ros != null && this.ros.isConnected) {
                this.ros.getNodes((data) => { }, (error) => { })
            }
        }, 10000)
        $('#start-rb1-form').on('submit', this.handleRB1Start);
        $('#stop-rb1-form').on('submit', this.handleRB1Stop);
        $('#init-srvs-form').on('submit', this.handleServiceServers);
        $('#start-nav2-form').on('submit', this.handleStartNav2);

    },
})
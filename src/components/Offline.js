import { Container } from "react-bootstrap";

const Offline = () => {
    return (
        <Container>
            <h3>To connect in Linux ROS Enviornment</h3>
            <hr />
            <ul>
                <li>
                    <p>Open terminal run below command.</p>
                    <p>command : <code>roscore</code></p>
                </li>
                <li>
                    <p>Open another terminal and run below command.</p>
                    <p>command : <code>rosrun turtlesim turtlesim_node</code></p>
                </li>
                <li>
                    <p>Run ROSBridge Websocket Server via another terminal</p>
                    <p>command : <code>roslaunch rosbridge_server rosbridge_websocket.launch</code></p>
                </li>
                <li>
                    <p>Write <code>ws://localhost:9090</code> or <code>ws://127.0.0.1:9090</code> it in our website's control section's websocket server address field.</p>
                </li>
                <li>
                    <p>controller will be enabled as soon as you click <code>connect</code> button.</p>
                </li>
                <li>
                    <p>Put <code>/turtle1/cmd_vel</code> in the Topic Name to use controller.</p>
                </li>
                <li>
                    <p>You are good to go.</p>
                </li>
                <li>
                    <p>You can see your robot moving the constructsim's simulator.</p>
                </li>
                
            </ul>
        </Container>
    );
}

export default Offline;
import { Container } from "react-bootstrap";

const Online = () => {
    return (
        <Container>
            <h3>To connect in Online Mode</h3>
            <hr />
            <ul>
                <li>
                    <p>Go to <a href="https://app.theconstructsim.com/" target="_blank" rel="noreferrer">Constructsim</a>.</p>
                </li>
                <li>
                    <p>create an account.</p>
                </li>
                <li>
                    <p>Search Project "<code>Developing Web Interfaces for ROS</code>" in the searchbar.</p>
                </li>
                <li>
                    <p>Open the project you will see termial, Documentation, IDE and simulator.</p>
                    <p>we will use termial and simulator.</p>
                </li>
                <li>
                    <p>Run ROSBridge Websocket Server via terminal</p>
                    <p>Command : <code>roslaunch course_web_dev_ros web.launch</code></p>
                </li>
                <li>
                    <p>Run the below command to get server link.</p>
                    <p>Command : <code>rosbridge_address</code></p>
                </li>
                <li>
                    <p>copy the link and paste it in our website's control section's websocket server address field.</p>
                </li>
                <li>
                    <p>controller will be enabled as soon as you click <code>connect</code> button.</p>
                </li>
                <li>
                    <p>Put <code>/cmd_vel</code> in the Topic Name to use controller.</p>
                </li>
                <li>
                    <p>You are good to go.</p>
                </li>
                <li>
                    <p>You can see your robot moving in the constructsim's simulator.</p>
                </li>
            </ul>
        </Container>
    );
}

export default Online;
import "bootstrap/dist/css/bootstrap.min.css";
import { Jumbotron } from "react-bootstrap";

function Home() {

	return (
		<Jumbotron style={{ marginBottom: '0px' }}>
			<h1>Hello from ROSDS!</h1>
			<hr />
			<h2>Go to <a href="https://app.theconstructsim.com/" target="_blank" rel="noreferrer">Constructsim</a></h2>
			<ul>
				<li>
					<p>create account</p>
				</li>
				<li>
					<p>Go to Project</p>
				</li>
				<li>
					<p>create  a new Project</p>
				</li>
				<li>
					<p>Run the Project</p>
				</li>
				<li>
					<p>Run ROSBridge Websocket Server via terminal</p>
					<p>command:<code> roslaunch rosbridge_server rosbridge_websocket.launch</code></p>
				</li>
			</ul>
		</Jumbotron>
	);
}

export default Home;

import "bootstrap/dist/css/bootstrap.min.css";
import { userStore } from './storage/store';
import * as action_type from "./storage/actiontype";
import { useSelector } from 'react-redux';

function App() {

	const log = useSelector(state => state.logs);
	const connected = useSelector(state => state.connected);

	// helper methods to connect to ROS
	function connect() {
		userStore.dispatch({
			type: action_type.CONNECT,
			payload: {
				ws_address: 'ws://13.115.59.232:9090'
			}
		});
	}

	function disconnect() {
		userStore.dispatch({
			type: action_type.DISCONNECT
		});
	};

	function forward() {
		userStore.dispatch({
			type: action_type.FORWARD
		});
		userStore.dispatch({
			type: action_type.SET_TOPIC
		});
	}

	function backward() {
		userStore.dispatch({
			type: action_type.BACKWARD
		});
		userStore.dispatch({
			type: action_type.SET_TOPIC
		});
	}

	function stop() {
		userStore.dispatch({
			type: action_type.STOP
		});
		userStore.dispatch({
			type: action_type.SET_TOPIC
		});
	}

	function turnleft() {
		userStore.dispatch({
			type: action_type.TURNLEFT
		});
		userStore.dispatch({
			type: action_type.SET_TOPIC
		});
	}

	function turnright() {
		userStore.dispatch({
			type: action_type.TURNRIGHT
		});
		userStore.dispatch({
			type: action_type.SET_TOPIC
		});
	}
	return (
		<div className="App">
			<div className="jumbotron" style={{marginBottom: '0px'}}>
				<h1>Hello from ROSDS!</h1>
				<hr/>
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
			</div>
			
			<div className="row" style={{ maxHeight: '200px', marginRight: '0px'}}>
				<div className="col-md-6">
					<h3>Connection status</h3>
					{ connected && <p className="text-success">Connected!</p> }
					{ !connected && <p className="text-danger" >Not connected!</p>}
					<label>Websocket server address</label>
					<input type="text"/>
					<br />

					{!connected &&
						<button className="btn btn-success" onClick={connect}>
							Connect!
						</button>
					}
					{connected &&
						<button className="btn btn-danger" onClick={disconnect}>
							Disconnect!
						</button>
					}
				</div>
				<div className="col-md-6" style={{maxHeight: '200px', overflow: 'auto'}}>
					<h3>Log messages</h3>
					<div>
						<p>{ log }</p>
					</div>
				</div>
			</div>

			<hr/>

				<div className="row">
					<div className="col-md-12 text-center">
						<h5>Commands</h5>
					</div>

					<div className="col-md-12 text-center">
					<button onClick={forward} className="btn btn-primary" disabled={!connected}>Go forward</button>
					<br/>
            </div>

						<div className="col-md-4 text-center">
					<button onClick={turnleft} className="btn btn-primary" disabled={!connected}>Turn left</button>
            </div>
					<div className="col-md-4 text-center">
					<button onClick={stop} className="btn btn-danger" disabled={!connected}>Stop</button>
					<br/>
            </div>
						<div className="col-md-4 text-center">
					<button onClick={turnright} className="btn btn-primary" disabled={!connected}>Turn right</button>
            </div>

					<div className="col-md-12 text-center">
					<button onClick={backward} className="btn btn-primary" disabled={!connected}>Go backward</button>
				</div>
			</div>
		</div>
	);
}

export default App;

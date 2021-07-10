import "bootstrap/dist/css/bootstrap.min.css";
import { userStore } from './storage/store';
import * as action_type from "./storage/actiontype";
import ROSLIB from 'roslib';
import { useSelector } from "react-redux";
import { useEffect, useState } from 'react';

function App() {

	let logs = useSelector(state => state.logs);
	let connected = useSelector(state => state.connected);
	const [ws, setWS] = useState('');

	useEffect(() => {
		// wss://i-0c6354da7096e9e8c.robotigniteacademy.com/a85a3c99-376f-42cd-a5e1-ca15e5f92ed7/rosbridge/
	}, [ws]);

	const connect = () => {
		let ros = null;
		ros = new ROSLIB.Ros({
			url: ws
		});

		if (ros !== null) {
			ros.on('connection', () => {
				userStore.dispatch({
					type: action_type.CONNECT,
					payload: {
						ros: ros
					}
				});
			});
			ros.on('error', (error) => {
				userStore.dispatch({
					type: action_type.ERROR,
					payload: {
						error: error
					}
				});
			});
			ros.on('close', () => {
				userStore.dispatch({
					type: action_type.DISCONNECT
				});
			});
		}
	}

	const disconnect = () => {
		let ros = userStore.getState().ros;
		if (ros !== null) {
			ros.close();
		}
	};

	const forward = () => {
		userStore.dispatch({
			type: action_type.SET_TOPIC
		});
		userStore.dispatch({
			type: action_type.FORWARD
		});

		let topic = userStore.getState().topic;
		let message = userStore.getState().message;

		if (topic !== null) {
			topic.publish(message);
		}
	}

	const backward = () => {
		userStore.dispatch({
			type: action_type.SET_TOPIC
		});
		userStore.dispatch({
			type: action_type.BACKWARD
		});

		let topic = userStore.getState().topic;
		let message = userStore.getState().message;

		if (topic !== null) {
			topic.publish(message);
		}
	}

	const stop = () => {
		userStore.dispatch({
			type: action_type.SET_TOPIC
		});
		userStore.dispatch({
			type: action_type.STOP
		});

		let topic = userStore.getState().topic;
		let message = userStore.getState().message;

		if (topic !== null) {
			topic.publish(message);
		}
	}

	const turnleft = () => {
		userStore.dispatch({
			type: action_type.SET_TOPIC
		});
		userStore.dispatch({
			type: action_type.TURNLEFT
		});

		let topic = userStore.getState().topic;
		let message = userStore.getState().message;

		if (topic !== null) {
			topic.publish(message);
		}
	}

	const turnright = () => {
		userStore.dispatch({
			type: action_type.SET_TOPIC
		});
		userStore.dispatch({
			type: action_type.TURNRIGHT
		});

		let topic = userStore.getState().topic;
		let message = userStore.getState().message;

		if (topic !== null) {
			topic.publish(message);
		}
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
					<input type="text" name='ws_address' value={ws} onChange={(e) => { setWS(e.target.value) }}/>
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
						<ul>
							{
								(Array.isArray(logs)) &&
								logs.map((log, i) => {
									return (
										<li key={i}>{log}</li>
									);
								})
							}
						</ul>
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

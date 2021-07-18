import { userStore } from '../storage/store';
import * as action_type from "../storage/actiontype";
import ROSLIB from 'roslib';
import { useSelector } from "react-redux";
import { useEffect, useState } from 'react';
import { Button, Container, TextField } from '@material-ui/core';
import { Jumbotron, Row } from 'react-bootstrap';

const Control = () => {
    let connected = useSelector(state => state.connected);
    const [ws, setWS] = useState(userStore.getState().ws_address);

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
                        ros: ros,
                        ws: ws
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

        <Jumbotron style={{ marginBottom: '0px' }}>
            <div className="row" style={{ maxHeight: '200px', marginRight: '0px' }}>
                <div className="col-12">
                    <Container>
                        <h3>Connection Status:</h3>
                        {
                            connected
                                ?
                                <p className="text-success">
                                    Connected
                                </p>
                                :
                                <p className="text-danger" >
                                    Not connected
                                </p>
                        }
                    </Container>
                    <hr />

                    <Container>
                        <Row>
                            <TextField
                                id="ws_address"
                                label="Websocket server address"
                                value={ ws === null ? '' : ws }
                                onChange={(e) => setWS(e.target.value)}
                                variant="outlined"
                            />
                        </Row>
                        <Row>
                            {
                                connected ?
                                    <Button variant="contained" color="secondary" onClick={disconnect}>
                                        Disconnect
                                    </Button>
                                    :
                                    <Button variant="contained" color="primary" onClick={connect}>
                                        Connect
                                    </Button>
                            }
                        </Row>
                    </Container>
                </div>
            </div>

            <hr />

            <div className="row">
                <div className="col-md-12 text-center">
                    <h5>Commands</h5>
                </div>

                <div className="col-md-12 text-center">
                    <Button variant="contained" color="primary" onClick={forward} disabled={!connected}>
                        Go forward
                    </Button>
                    <br />
                </div>

                <div className="col-md-4 text-center">
                    <Button variant="contained" color="primary" onClick={turnleft} disabled={!connected}>
                        Turn left
                    </Button>
                </div>
                <div className="col-md-4 text-center">
                    <Button variant="contained" color="secondary" onClick={stop} disabled={!connected}>
                        Stop
                    </Button>
                    <br />
                </div>
                <div className="col-md-4 text-center">
                    <Button variant="contained" color="primary" onClick={turnright} disabled={!connected}>
                        Turn right
                    </Button>
                </div>

                <div className="col-md-12 text-center">
                    <Button variant="contained" color="primary" onClick={backward} disabled={!connected}>
                        Go backward
                    </Button>
                </div>
            </div>
        </Jumbotron>
    );
}

export default Control;
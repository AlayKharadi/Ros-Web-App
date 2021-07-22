import { userStore } from '../storage/store';
import * as action_type from "../storage/actiontype";
import ROSLIB from 'roslib';
import { useSelector } from "react-redux";
import { useEffect, useState } from 'react';
import { Button, Container, TextField } from '@material-ui/core';
import { Col, Row } from 'react-bootstrap';

const Control = () => {
    let connected = useSelector(state => state.connected);
    const [ws, setWS] = useState(userStore.getState().ws_address);
    const [topicName, setTopicName] = useState(userStore.getState().topic === null ? "" : userStore.getState().topic.name);

    useEffect(() => {
        // link for the web
    }, [ws]);

    useEffect(() => {
        // /cmd_vel 
        // /turtle1/cmd_vel
    }, [topicName]);

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
            type: action_type.SET_TOPIC,
            payload: {
                topicName: topicName
            }
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
            type: action_type.SET_TOPIC,
            payload: {
                topicName: topicName
            }
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
            type: action_type.SET_TOPIC,
            payload: {
                topicName: topicName
            }
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
            type: action_type.SET_TOPIC,
            payload: {
                topicName: topicName
            }
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
            type: action_type.SET_TOPIC,
            payload: {
                topicName: topicName
            }
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

        <div className="container" style={{ marginBottom: '0px' }}>
            <Row>
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
                    <hr />
                </Container>

                <Container>
                    <Row>
                        <Col>
                            <TextField
                                id="ws_address"
                                type="text"
                                name="ws_address"
                                variant="filled"
                                label="Websocket server address:"
                                value={ws === null ? '' : ws}
                                onChange={(e) => setWS(e.target.value)}
                                fullWidth
                            />
                        </Col>
                    </Row>

                    <Row>
                        <Col>
                            {
                                connected ?
                                    <Button variant="contained" color="secondary" onClick={disconnect} style={{marginTop: "10px"}}>
                                        Disconnect
                                    </Button>
                                    :
                                    <Button variant="contained" color="primary" onClick={connect} style={{ marginTop: "10px" }}>
                                        Connect
                                    </Button>
                            }
                        </Col>
                    </Row>

                    <hr />

                    <Row>
                        <Col>
                            <TextField
                                id="topic_name"
                                type="text"
                                name="topic_name"
                                variant="filled"
                                label="Topic Name:"
                                value={topicName === null ? '' : topicName}
                                onChange={(e) => setTopicName(e.target.value)}
                                fullWidth
                            />
                        </Col>
                    </Row>
                </Container>
            </Row>

            <hr />

            <Row>
                <Container>
                    <Row>
                        <Col md={12} className="text-center">
                            <h5>
                                Commands
                            </h5>
                        </Col>
                    </Row>

                    <Row>
                        <Col md={12} className="text-center">
                            <Container>
                                <Button variant="contained" color="primary" onClick={forward} disabled={!connected}>
                                    Go forward
                                </Button>
                            </Container>
                            <br />
                        </Col>
                    </Row>

                    <Row>
                        <Col md={4} className="text-center">
                            <Container>
                                <Button variant="contained" color="primary" onClick={turnleft} disabled={!connected}>
                                    Turn left
                                </Button>
                            </Container>
                        </Col>

                        <Col md={4} className="text-center">
                            <Container>
                                <Button variant="contained" color="secondary" onClick={stop} disabled={!connected}>
                                    Stop
                                </Button>
                            </Container>
                            <br />
                        </Col>

                        <Col md={4} className="text-center">
                            <Container>
                                <Button variant="contained" color="primary" onClick={turnright} disabled={!connected}>
                                    Turn right
                                </Button>
                            </Container>
                        </Col>
                    </Row>


                    <Row>
                        <Col md={12} className="text-center">
                            <Container>
                                <Button variant="contained" color="primary" onClick={backward} disabled={!connected}>
                                    Go backward
                                </Button>
                            </Container>
                        </Col>
                    </Row>
                </Container>
            </Row>
        </div>
    );
}

export default Control;
import style from '../styles/style.module.css';
import { useSelector } from "react-redux";
import { Alert, AlertTitle } from '@mui/material';
import { Container } from "react-bootstrap";

const Logs = () => {
    const logs = useSelector(state => state.logs);
    
    return (
        <Container className={style.container}>
            <h3>Log messages:</h3>
            <div>
                <ul>
                    {
                        (Array.isArray(logs)) &&
                        logs.map((log, i) => {
                            if (log.type === 'connect') {
                                return (
                                    <Alert severity="success" key={i}>
                                        <AlertTitle>
                                            Connect
                                        </AlertTitle>
                                        {log.text}
                                    </Alert>
                                );
                            } else if (log.type === 'disconnect') {
                                return (
                                    <Alert severity="warning" key={i}>
                                        <AlertTitle>
                                            Disconnect
                                        </AlertTitle>
                                        {log.text}
                                    </Alert>
                                );
                            } else {
                                return (
                                    <Alert severity="error" key={i}>
                                        <AlertTitle>
                                            Error
                                        </AlertTitle>
                                        {log.text}
                                    </Alert>
                                );
                            }
                        })
                    }
                </ul>
            </div>
        </Container>
    );
}

export default Logs;
import { useSelector } from "react-redux";
import { Jumbotron } from 'react-bootstrap';
import { Alert, AlertTitle } from '@material-ui/lab';

const Logs = () => {
    let logs = useSelector(state => state.logs);
    
    return (
        <Jumbotron style={{ marginBottom: '0px' }}>
            <h3>Log messages</h3>
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
        </Jumbotron>
    );
}

export default Logs;
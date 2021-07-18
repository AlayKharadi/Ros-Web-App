import { BrowserRouter, Switch, Redirect, Route } from "react-router-dom";
import Control from "./components/Control";
import CustomNavbar from "./components/CustomNavbar";
import Home from "./components/Home";
import Logs from "./components/Logs";

const App = () => {
    return (
        <BrowserRouter>
            <CustomNavbar></CustomNavbar>
            <Switch>
                <Route exact path='/Home'>
                    <Home></Home>
                </Route>
                <Route exact path='/Control'>
                    <Control></Control>
                </Route>
                <Route exact path='/Logs'>
                    <Logs></Logs>
                </Route>
                <Route exact path='/'>
                    <Redirect to='/Home'></Redirect>
                </Route>
            </Switch>
        </BrowserRouter>
    );
}

export default App;
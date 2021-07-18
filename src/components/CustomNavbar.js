import { useState } from 'react';
import { Link } from 'react-router-dom';
import { Container, Nav, Navbar } from 'react-bootstrap';

const CustomNavbar = () => {
    const [active, setActive] = useState(window.location.pathname === '/' ? 0 : false);

    const navlinks = [
        { id: 0, path: '/Home', name: 'Home' },
        { id: 1, path: '/Control', name: 'Control' },
        { id: 2, path: '/Logs', name: 'Logs' }
    ]

    return (
        <Navbar collapseOnSelect expand="lg" bg="dark" variant="dark" sticky="top">
            <Container>
                <Link className="navbar-brand" to={navlinks[0].path} onClick={() => setActive(navlinks[0].id)}>
                    ROS
                </Link>
                <Navbar.Toggle aria-controls="responsive-navbar-nav" />
                <Navbar.Collapse id="responsive-navbar-nav">
                    <Nav>
                        {
                            navlinks.map(navlink => {
                                return (
                                    <Link key={navlink.id} className={"nav-link " + ((navlink.id === active || navlink.path === window.location.pathname) ? " active" : "")} to={navlink.path} onClick={() => setActive(navlink.id)}>
                                        {navlink.name}
                                    </Link>
                                );
                            })
                        }
                    </Nav>
                </Navbar.Collapse>
            </Container>
        </Navbar>
    );
}

export default CustomNavbar;
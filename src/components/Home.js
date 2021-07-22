import { useState } from "react";
import { Tabs, Tab } from "react-bootstrap";
import Offline from "./Offline";
import Online from "./Online";

function Home() {
	const [key, setKey] = useState('online');

	return (
		<div className="container" style={{ marginBottom: '0px' }}>
			<h3>Introduction:</h3>
			<ul>
				<li>We have created an online website through which you can control the ROS robot.</li>
				<li>Below you have instructions for how to use the website both in online or offline mode.</li>
				<li>you can find controller in control section.</li>
				<li>logs regarding the connection process are in logs section.</li>
				<li>This is part of our Institute's project. If you want to contribute to this project this is the link <a href="https://github.com/AlayKharadi/Ros-Web-App" target="_blank" rel="noreferrer">Github Project</a>. feel free to add something new to this project.</li>

			</ul>
			<Tabs
				id="controlled-tab-example"
				activeKey={key}
				onSelect={(k) => setKey(k)}
				className="mb-3"
			>
				<Tab eventKey="online" title="Online">
					<Online></Online>
				</Tab>
				<Tab eventKey="offline" title="Offline">
					<Offline></Offline>
				</Tab>
			</Tabs>
			</div>
	);
}

export default Home;

import { lazy, Suspense, useState } from "react";
import { Tabs, Tab, Container, Spinner } from "react-bootstrap";
const Offline = lazy(() => import('./Offline'));
const Online = lazy(() => import('./Online'));

function Home() {
	const [key, setKey] = useState('online');

	return (
		<Container style={{ marginBottom: '0px', padding: '2em' }}>
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
					<Suspense fallback={<Spinner />}>
						<Online></Online>
					</Suspense>
				</Tab>
				<Tab eventKey="offline" title="Offline">
					<Suspense fallback={<Spinner />}>
						<Offline></Offline>
					</Suspense>
				</Tab>
			</Tabs>
		</Container>
	);
}

export default Home;

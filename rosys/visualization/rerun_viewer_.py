from __future__ import annotations

import rerun as rr
import rerun.blueprint as rrb
from nicegui import ui

from .rerun_logger import RerunLogger


class RerunViewer(ui.html):
    """A NiceGUI element that embeds the Rerun web viewer.

    This element creates an iframe that loads the Rerun web viewer served by
    a RerunLogger. The viewer automatically connects to the gRPC data stream
    and displays logged data in real-time.

    For replay mode, use `load_recording()` to load an RRD file URL.

    :param logger: The RerunLogger instance that serves the web viewer
    :param height: CSS height of the viewer (default: '600px')
    :param width: CSS width of the viewer (default: '100%')
    """

    def __init__(
        self,
        logger: RerunLogger,
    ) -> None:
        self.logger = logger

        super().__init__(sanitize=False)
        iframe_id = f'rerun-viewer-{self.id}'
        self.set_content(f'''
                <iframe
                    id="{iframe_id}"
                    style="width: 100%; height: 100%; border: none;"
                    allow="cross-origin-isolated"
                ></iframe>
            ''')

        # Set the iframe src after the page is ready (needs client-side hostname)
        # The url parameter tells the viewer to connect to the gRPC server for live data
        ui.timer(0.1, lambda: ui.run_javascript(f'''
            const iframe = document.getElementById('{iframe_id}');
            if (iframe) {{
                const host = window.location.hostname;
                const grpcUrl = encodeURIComponent(`rerun+http://${{host}}:{logger.grpc_port}/proxy`);
                iframe.src = `http://${{host}}:{logger.web_port}?url=${{grpcUrl}}`;
            }}
        '''), once=True)

    def load_recording(self, rrd_url: str) -> None:
        """Load an RRD recording file into the viewer.

        The URL can be:
        - A relative URL served by the rosys application
        - An absolute URL to an RRD file hosted elsewhere

        :param rrd_url: URL to the RRD file to load
        """
        iframe_id = f'rerun-viewer-{self.id}'
        ui.run_javascript(f'''
            const iframe = document.getElementById('{iframe_id}');
            if (iframe) {{
                const host = window.location.hostname;
                const rrdUrl = encodeURIComponent('{rrd_url}');
                iframe.src = `http://${{host}}:{self.logger.web_port}?url=${{rrdUrl}}`;
            }}
        ''')

    def set_time_window(self, seconds: float = 20.0) -> None:
        """Configure the viewer to show only the last N seconds of data."""

        time_range = rrb.VisibleTimeRange(
            timeline='rosys_time',
            start=rrb.TimeRangeBoundary.cursor_relative(seconds=-seconds),
            end=rrb.TimeRangeBoundary.cursor_relative(),
        )

        blueprint = rrb.Blueprint(
            rrb.Spatial2DView(origin='/', time_ranges=[time_range]),
            rrb.Spatial3DView(origin='/', time_ranges=[time_range]),
        )
        rr.send_blueprint(blueprint)

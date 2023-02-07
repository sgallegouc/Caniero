#cython: language_level=3, boundscheck=False
import multiprocessing as mp
from enum import Enum
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
Gst.init(None)

'''Konwn issues

* if format changes at run time system hangs
'''

class StreamMode(Enum):
    INIT_STREAM = 1
    SETUP_STREAM = 1
    READ_STREAM = 2


class StreamCommands(Enum):
    FRAME = 1
    ERROR = 2
    HEARTBEAT = 3
    RESOLUTION = 4
    STOP = 5


class StreamCapture(mp.Process):

    def __init__(self, link, stop, outQueue, framerate):
        """
        Initialize the stream capturing process
        link - rstp link of stream
        stop - to send commands to this process
        outPipe - this process can send commands outside
        """

        super().__init__()
        self.streamLink = link
        self.stop = stop
        self.outQueue = outQueue
        self.framerate = framerate
        self.currentState = StreamMode.INIT_STREAM
        self.pipeline = None
        self.source = None
        self.decode = None
        self.convert = None
        self.sink = None
        self.image_arr = None
        self.newImage = False
        self.frame1 = None
        self.frame2 = None
        self.num_unexpected_tot = 400
        self.unexpected_cnt = 0

    def new_buffer_d(self, sink, _):
        sample = sink.emit("pull-sample")
        buffer = sample.get_buffer()  # Gst.Buffer

        caps_format = sample.get_caps().get_structure(0)  # Gst.Structure

        # GstVideo.VideoFormat
        frmt_str = caps_format.get_value('format') 

        w, h = caps_format.get_value('width'), caps_format.get_value('height')
        c = 1 # GstUtils.get_num_channels(video_format)

        # print (w, h, video_format, type(buffer))

        array = np.ndarray(shape=(h, w, c), buffer=buffer.extract_dup(0, buffer.get_size()), dtype=np.uint16)
        
        self.image_arr = array
        self.newImage = True

        return Gst.FlowReturn.OK

    def gst_to_opencv(self, sample):
        buf = sample.get_buffer()
        caps = sample.get_caps()

        # Print Height, Width and Format
        # print(caps.get_structure(0).get_value('format'))
        # print(caps.get_structure(0).get_value('height'))
        # print(caps.get_structure(0).get_value('width'))

        arr = np.ndarray(
            (caps.get_structure(0).get_value('height'),
             caps.get_structure(0).get_value('width'),
             3),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8)
        return arr

    def new_buffer(self, sink, _):
        sample = sink.emit("pull-sample")
        arr = self.gst_to_opencv(sample)
        self.image_arr = arr
        self.newImage = True
        return Gst.FlowReturn.OK

    def __del__(self):
        print('terminating cam pipe (__del__)')
        # self.stop.set()
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)

    def run(self):

        is_depth = 'depth' in self.streamLink
        pipe = 'rtspsrc name=m_rtspsrc ! rtpgstdepay ! videoconvert ! appsink name=m_appsink' if is_depth else 'rtspsrc name=m_rtspsrc ! rtph264depay name=m_rtph264depay ! avdec_h264 name=m_avdech264 ! videoconvert name=m_videoconvert ! videorate name=m_videorate ! appsink name=m_appsink'

        # Create the empty pipeline
        self.pipeline = Gst.parse_launch(
            pipe)

        # source params
        self.source = self.pipeline.get_by_name('m_rtspsrc')
        self.source.set_property('latency', 0)
        self.source.set_property('location', self.streamLink)
        self.source.set_property('protocols', 'tcp')
        self.source.set_property('retry', 50)
        self.source.set_property('timeout', 50)
        self.source.set_property('tcp-timeout', 5000000)
        self.source.set_property('drop-on-latency', 'true')

        # sink params
        self.sink = self.pipeline.get_by_name('m_appsink')
        self.sink.set_property('max-buffers', 5)
        self.sink.set_property('drop', 'true')
        self.sink.set_property('emit-signals', True)

        if is_depth:
            self.sink.connect("new-sample", self.new_buffer_d, None)
        else:
            # decode params
            self.decode = self.pipeline.get_by_name('m_avdech264')
            self.decode.set_property('max-threads', 2)
            self.decode.set_property('output-corrupt', 'false')

            # convert params
            self.convert = self.pipeline.get_by_name('m_videoconvert')

            #framerate parameters
            self.framerate_ctr = self.pipeline.get_by_name('m_videorate')
            self.framerate_ctr.set_property('max-rate', self.framerate/1)
            self.framerate_ctr.set_property('drop-only', 'true')

            # sink params
            self.sink = self.pipeline.get_by_name('m_appsink')
            self.sink.set_property('max-lateness', 500000000)

            caps = Gst.caps_from_string(
                'video/x-raw, format=(string){BGR, GRAY8}; video/x-bayer,format=(string){rggb,bggr,grbg,gbrg}')
            self.sink.set_property('caps', caps)

            if not self.source or not self.sink or not self.pipeline or not self.decode or not self.convert:
                print("Not all elements could be created.")
                print (self.source, self.sink, self.pipeline, self.decode, self.convert)
                self.stop.set()

            self.sink.connect("new-sample", self.new_buffer, self.sink)

        # Start playing
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("Unable to set the pipeline to the playing state.")
            self.stop.set()

        # Wait until error or EOS
        bus = self.pipeline.get_bus()

        while True:

            if self.stop.is_set():
                print('Stopping CAM Stream by main process')
                break

            message = bus.timed_pop_filtered(10000, Gst.MessageType.ANY)
            # print "image_arr: ", image_arr
            if self.image_arr is not None and self.newImage is True:

                if not self.outQueue.full():

                    # print("\r adding to queue of size{}".format(self.outQueue.qsize()), end='\r')
                    self.outQueue.put((StreamCommands.FRAME, self.image_arr), block=False)

                self.image_arr = None
                self.unexpected_cnt = 0


            if message:
                if message.type == Gst.MessageType.ERROR:
                    err, debug = message.parse_error()
                    print("Error received from element %s: %s" % (
                        message.src.get_name(), err))
                    print("Debugging information: %s" % debug)
                    break
                elif message.type == Gst.MessageType.EOS:
                    print("End-Of-Stream reached.")
                    break
                elif message.type == Gst.MessageType.STATE_CHANGED:
                    if isinstance(message.src, Gst.Pipeline):
                        old_state, new_state, pending_state = message.parse_state_changed()
                        print("Pipeline state changed from %s to %s." %
                              (old_state.value_nick, new_state.value_nick))
                else:
                    print("Unexpected message received: ", message.type)
                    self.unexpected_cnt = self.unexpected_cnt + 1
                    if self.unexpected_cnt == self.num_unexpected_tot:
                        break

        print('terminating cam pipe')
        self.stop.set()
        self.pipeline.set_state(Gst.State.NULL)
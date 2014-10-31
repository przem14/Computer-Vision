Computer-Vision
===============

<h3>Installation OpenCV</h3>
<dl type="1">
    <dt> Mac OSX </dt>
    <dd>
      <ol type="1">
        <li>Open terminal</li>
        <li>Install brew (http://brew.sh)</li>
        <li>Use command: <b>brew tap homebrew/science</b></li>
        <li>Use command: <b>brew install opencv</b></li>
        <li>You can find OpenCV in <b>/usr/local/Cellar/opencv/2.4.9/</b></li>
      </ol>
    </dd>
</dl>
<dl type="1">
    <dt> Ubuntu </dt>
    <dd>
      <ol type="1">
        <li>Make sure that you have installed: <i>GCC 4.4.x or later, CMake 2.6 or higher, Git, GTK+2.x or higher, pkg-config, Python 2.6 or later and Numpy 1.5 or later, ffmpeg or libav development packages</i></li>
	<li>Open terminal</li>
	<li>Use command: <b>git clone https://github.com/Itseez/opencv.git</b></li>
	<li>Go to the opencv folder cloned from git</li>
	<li>Use command: <b>cmake -D CMAKE_BUILD_TYPE=RELEASE -D BUILD_NEW_PYTHON_SUPPORT=ON -D CMAKE_INSTALL_PREFIX=/usr/local ..</b></li>
	<li>Use command: <b>make</b></li>
	<li>Use command: <b>sudo make install</b></li>
      </ol>
    </dd>
</dl>

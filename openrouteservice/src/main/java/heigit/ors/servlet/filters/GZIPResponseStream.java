package heigit.ors.servlet.filters;

import java.io.IOException;
import java.util.zip.GZIPOutputStream;

import javax.servlet.ServletOutputStream;
import javax.servlet.WriteListener;
import javax.servlet.http.HttpServletResponse;

import heigit.ors.io.ByteArrayOutputStreamEx;

class GZIPResponseStream extends ServletOutputStream { 
	private ByteArrayOutputStreamEx _bufferStream = null;
	private GZIPOutputStream _gzipstream = null;
	private ServletOutputStream _outputStream = null;
	private boolean _closed = false;
	private HttpServletResponse _response = null;

	public GZIPResponseStream(HttpServletResponse response) throws IOException {
		super();
		
		this._response = response;
		this._outputStream = response.getOutputStream();
		_bufferStream = new ByteArrayOutputStreamEx();
		_gzipstream = new GZIPOutputStream(_bufferStream);
	}

	public void close() throws IOException {
		if (_closed) 
			throw new IOException("This output stream has already been closed");
		
		_gzipstream.finish();

		byte[] bytes = _bufferStream.getBuffer();
		int bytesLength = _bufferStream.size();
				
		_response.setContentLength(bytesLength); 
        _response.addHeader("Content-Encoding", "gzip");

        _outputStream.write(bytes, 0, bytesLength);
        _outputStream.close();
		_closed = true;
	}

	public void flush() throws IOException {
		if (_closed) 
			throw new IOException("Cannot flush a closed output stream");
		
		_gzipstream.flush();
	}

	public void write(int b) throws IOException {
		if (_closed) 
			throw new IOException("Cannot write to a closed output stream");
		
		_gzipstream.write((byte)b);
	}

	public void write(byte b[]) throws IOException {
		write(b, 0, b.length);
	}

	public void write(byte b[], int off, int len) throws IOException {
		if (_closed) 
			throw new IOException("Cannot write to a closed output stream");
		
		_gzipstream.write(b, off, len);
	}

	public boolean closed() {
		return (this._closed);
	}

	public void reset() {

	}

	@Override
	public boolean isReady() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void setWriteListener(WriteListener arg0) {
		// TODO Auto-generated method stub

	}
}
"""One page of inherited, non-actuating release diagnostics. No hot-path IO."""
import mmap
import fcntl
import os
import stat
import struct
import sys
import tempfile

PAGE=4096
SLOTS=32
MAGIC=b'RBWREL1\0'
HEADER=struct.Struct('<8sIIIIQQ')  # version, committed count, truncated, flags, PID, generation
ROW=struct.Struct('<IIQ')
COUNT_OFFSET=12
TRUNCATED_OFFSET=16
FLAGS_OFFSET=20
STEPS=('guard_releasing','set_to_damping','pop_left','destroy_left','retire_left',
       'pop_right','destroy_right','retire_right','guard_disarm','close','quiesce_left','quiesce_right')
STATUSES=('started','returned','failed')
STEP_IDS={name:i+1 for i,name in enumerate(STEPS)}
STATUS_IDS={name:i+1 for i,name in enumerate(STATUSES)}
LINUX=sys.platform=='linux'
# Linux UAPI values; some Conda builds omit these Python symbols.
F_ADD_SEALS=getattr(fcntl,'F_ADD_SEALS',1033)
F_GET_SEALS=getattr(fcntl,'F_GET_SEALS',1034)


def create_memfd(name):
    """Use the same kernel RAM fd even when Python was built with old headers."""
    if not LINUX:raise OSError('memfd requires Linux')
    if not isinstance(name,str) or not name or '\0' in name:raise ValueError('memory fd name required')
    flags=0x0001|0x0002  # MFD_CLOEXEC | MFD_ALLOW_SEALING
    native=getattr(os,'memfd_create',None)
    if native is not None:return native(name,flags)
    import ctypes
    import errno
    libc=ctypes.CDLL(None,use_errno=True)
    try:create=libc.memfd_create
    except AttributeError:raise OSError(errno.ENOSYS,'libc memfd_create unavailable; no disk fallback')from None
    create.argtypes=(ctypes.c_char_p,ctypes.c_uint);create.restype=ctypes.c_int
    fd=create(os.fsencode(name),flags)
    if fd<0:
        error=ctypes.get_errno();raise OSError(error,os.strerror(error))
    return fd


def memory_file(name, size):
    """Prefaulted, fixed-size Linux RAM fd; disk fallback is for non-Linux CPU tests only."""
    if type(size)is not int or not 0<size<=4*1024**2:raise ValueError('bounded memory size required')
    if LINUX:
        fd=create_memfd(name)
        file=os.fdopen(fd,'r+b',buffering=0)
    else:file=tempfile.TemporaryFile()
    try:
        os.fchmod(file.fileno(),0o600);file.truncate(size)
        with mmap.mmap(file.fileno(),size)as memory:memory[:]=bytes(size)
        if LINUX:
            fcntl.fcntl(file.fileno(),F_ADD_SEALS,4|2|1)  # GROW | SHRINK | SEAL
        return file
    except BaseException:
        file.close();raise


def archive_memory_file(file, path):
    """Call only after the writing child has exited; never from a control loop."""
    limit=os.fstat(file.fileno()).st_size
    count=min(os.lseek(file.fileno(),0,os.SEEK_CUR),limit)
    raw=os.pread(file.fileno(),count,0)
    with os.fdopen(os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600),'wb')as output:
        output.write(raw)
    return dict(bytes=len(raw),limit_bytes=limit,truncated=count>=limit,storage='bounded_memory_then_post_exit_archive')


class ReleaseTrace:
    def __init__(self,file,buffer,*,pid=0,generation=0):
        self.file,self.buffer,self.pid,self.generation=file,buffer,pid,generation
        self.fd=file.fileno();self.enabled=False;self.count=self.truncated=0

    @classmethod
    def create(cls):
        file=memory_file('workbench-release-trace',PAGE);buffer=None
        try:
            buffer=mmap.mmap(file.fileno(),PAGE)
            buffer[:]=bytes(PAGE)  # Allocate/touch the page before spawning the SDK process.
            HEADER.pack_into(buffer,0,MAGIC,1,0,0,0,0,0)
            return cls(file,buffer)
        except BaseException:
            if buffer is not None:buffer.close()
            file.close();raise

    @classmethod
    def inherit(cls,fd):
        if type(fd)is not int or fd<=2:raise ValueError('invalid release diagnostic descriptor')
        file=os.fdopen(fd,'r+b');buffer=None
        try:
            info=os.fstat(fd)
            if (not stat.S_ISREG(info.st_mode) or info.st_uid!=os.getuid()
                    or stat.S_IMODE(info.st_mode)!=0o600 or info.st_size!=PAGE):
                raise ValueError('invalid private release diagnostic descriptor')
            os.set_inheritable(fd,False)
            buffer=mmap.mmap(fd,PAGE);header=HEADER.unpack(buffer[:HEADER.size])
            pid=os.getpid()
            if header[:2]!=(MAGIC,1) or header[5] not in (0,pid):
                raise ValueError('release diagnostic identity differs')
            HEADER.pack_into(buffer,0,MAGIC,1,0,0,0,pid,header[6])
            return cls(file,buffer,pid=pid,generation=header[6])
        except BaseException:
            if buffer is not None:buffer.close()
            file.close();raise

    def begin(self):
        """Called only after the existing releasing ACK; not while armed."""
        self.count=self.truncated=0;self.generation+=1
        try:
            HEADER.pack_into(self.buffer,0,MAGIC,1,0,0,0,self.pid,self.generation)
            self.enabled=True
        except (ValueError,BufferError,struct.error):self.enabled=False

    def mark(self,step,status,monotonic_ns):
        if not self.enabled:return
        try:
            if step not in STEP_IDS or status not in STATUS_IDS or type(monotonic_ns)is not int or monotonic_ns<=0:
                struct.pack_into('<I',self.buffer,FLAGS_OFFSET,1);self.enabled=False;return
            if self.count==SLOTS:
                self.truncated=min(self.truncated+1,2**32-1)
                struct.pack_into('<I',self.buffer,TRUNCATED_OFFSET,self.truncated);return
            ROW.pack_into(self.buffer,HEADER.size+self.count*ROW.size,STEP_IDS[step],STATUS_IDS[status],monotonic_ns)
            self.count+=1
            struct.pack_into('<I',self.buffer,COUNT_OFFSET,self.count)  # Publish only a complete row.
        except (ValueError,BufferError,struct.error):self.enabled=False

    def snapshot(self,expected_pid,*,not_before_ns=None):
        """Copy a committed prefix after termination/disarm, never for admission."""
        result=dict(schema='workbench.release_trace.v1',supported=True,status='unknown',clock='host_monotonic')
        try:
            for _ in range(2):
                prefix=self.buffer[:HEADER.size]
                magic,version,count,truncated,flags,pid,generation=HEADER.unpack(prefix)
                if magic!=MAGIC or version!=1 or count>SLOTS or flags:
                    raise ValueError('release diagnostic header invalid')
                raw=self.buffer[HEADER.size:HEADER.size+count*ROW.size]
                if self.buffer[:HEADER.size]==prefix:break
            else:raise ValueError('release diagnostic snapshot changed')
            if pid not in (0,expected_pid) or count and pid!=expected_pid:
                raise ValueError('release diagnostic PID differs')
            events=[];previous=0
            for step,status,stamp in ROW.iter_unpack(raw):
                if not 1<=step<=len(STEPS) or not 1<=status<=len(STATUSES) or not stamp or stamp<previous:
                    raise ValueError('release diagnostic row invalid')
                events.append(dict(step=STEPS[step-1],status=STATUSES[status-1],monotonic_ns=stamp))
                previous=stamp
            if events and not_before_ns is not None and events[0]['monotonic_ns']<not_before_ns:
                raise ValueError('release diagnostic predates current release')
            result.update(status='partial' if truncated else 'observed' if events else 'not_started',
                source_pid=pid,generation=generation,events=events,truncated=truncated,
                last_confirmed_step=events[-1] if events else None)
        except (ValueError,BufferError,struct.error,IndexError) as error:result['error']=str(error)
        return result

    def close(self):
        self.enabled=False
        self.buffer.close();self.file.close()

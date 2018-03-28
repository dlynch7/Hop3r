# README
Raspberri Pi - client PC interface

## Getting connected
I assume you're using an Ethernet cable. If you're using WiFi instead, the steps are still similar. Make sure the Pi is on and connected to the wired (wireless) network.

1. Find your PC's IP address. In a terminal, type
```
hostname -I
```
On my computer, this returns 10.42.0.1.
2. Use `nmap` to find other devices on the wired (wireless) network. In the terminal, type
```
nmap -n -sP 10.42.0.255/24
```
This will return at least two addresses: your PC's address and the Pi's address (something like 10.42.0.67).

3. Now that you know the Pi's IP address, you can connect to it with `SSH` (and `sftp`, if you plan on transferring files). In the terminal, type
```
ssh pi@<the IP you found in step 2 above>
```
When prompted, enter the Pi's password.

## Running the Pi and client programs
Now that you have SSH'ed into the Pi, `cd` into `~/Embedded/client`. Open a new terminal on the client PC, and `cd` into `.../Hop3r/RaspberryPi/client`, where you should see `serial_basic.py`.

If you have modified anything of the Pi's code, you will need to compile it. In your SSH terminal, you should see the following files (among others) if you type `ls`:
* circ_buffer.c
* circ_buffer.h
* main.c
* Makefile

The Makefile automates the compilation process and saves you a lot of typing time. To remove all previously generated object files, type `make clean`. To recompile, simply type `make`. This will generate `main.a`, which is the executable program you must run by typing `./main.a`. But don't run it just yet!

The `main` and `serial_basic.py` employ a quick handshaking process wherein `serial_basic.py` gives `main` permission to start sending data. This handshake, along with data transmission, occurs over a serial port with a timeout.
You need two terminals open (they should still be open from earlier). First, in the terminal on your PC, type `python serial_basic.py`. Then (don't hesitate!) in the SSH terminal, type `./main.a`.

If all goes well, you should see a lot of numbers go scrolling down each window. The meaning of those numbers and how they got there is explained in the next section, below.

## A closer look at `main.c`
`main.c` is a multithreaded program. Well, really, it only uses two threads at the moment. The two threads are linked by a common data structure: a circular buffer.

### Circular buffers and asynchronous tasks

The circular buffer is basically just a fixed-length array. Data is written to the buffer at the _write index_, and data is read out from the buffer at the _read index_. The buffer is _circular_ because each index wraps around. Two blocks of code achieve this:
```c
// read index wraparound
if(read >= BUFLEN) {
  read = 0;
}
```

```c
// read index wraparound
if(write >= BUFLEN) {
  write = 0;
}
```

The circular buffer is a useful data structure for asynchronous reads and writes. Asynchrony is in turn useful for avoiding bottlenecks. The two threads in `main.c` are examples of asynchronous tasks. One thread, `CAN_thread`, puts data into the circular buffer (and does other things too), and the other thread, `UART_thread`, reads data from the circular buffer and sends it to the client PC via a UART.

### Mutexes

An important element in multi-threaded (or multi-process) applications is a _mutex_ (short for _mutual exclusion_), which keeps threads or processes from interfering with each other when using shared resources (e.g., data structures or hardware blocks). The two most important operations on a mutex are _locking_ and _unlocking_, and the basic rule is simply this: whichever thread locks the mutex must unlock it before any other threads try to lock it. Here are two examples from `main.c`:

1. CAN setup in `CAN_thread`:

```c
pthread_mutex_lock(&mutex1);
int s; // can raw socket
int nbytes;
struct sockaddr_can addr;
struct can_frame frame;
struct ifreq ifr;

/* open socket */
if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
  perror("socket");
  // TO-DO: ERROR HANDLING
  // return 1;
}

addr.can_family = AF_CAN;

strcpy(ifr.ifr_name, "can0");
if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
  perror("SIOCGIFINDEX");
  // TO-DO: ERROR HANDLING
  // return 1;
}
addr.can_ifindex = ifr.ifr_ifindex;

setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

if (bind(s, (struct sockaddr * )&addr, sizeof(addr)) < 0) {
  perror("bind");
  // TO-DO: ERROR HANDLING
  // return 1;
}

printf("CAN socket set up successfully!\n");
pthread_mutex_unlock(&mutex1);
```
Don't worry too much about what the code's doing, but notice that the very first line is a mutex lock, and the very last line is a mutex unlock. This prevents `UART_thread` from interfering with `CAN_thread` while it's setting up a CAN socket.

2. Buffer read in `UART_thread`:

```c
nextReadTime = millis() + periodms_read;
for (j = 0; j < BUFLEN;) {
  if (millis() > nextReadTime) {
    pthread_mutex_lock(&mutex1);
    bufferval = buffer_read();
    printf("data_buf[%d] = %d\tread = %d\twrite = %d\tempty = %d\tfull = %d\n",\
    j,bufferval,get_read_index(),get_write_index(),buffer_empty(),buffer_full());

    fflush(stdout);
    sprintf(writemsg,"%d\r\n",bufferval);
    serialPuts(serial_port, writemsg);
    nextReadTime += periodms_read;
    ++j;
    pthread_mutex_unlock(&mutex1);
  }
```

Here, a mutex lock & unlock is used to restrict access to the circular buffer while `UART_thread` is reading from it. The mutex is only locked for the duration of the read.

#### Caution when using mutexes:
Exercise caution when using mutexes. Make sure that if you lock a mutex, you know when and where you're unlocking it. This warning also applies to commenting out code that uses a mutex.

FROM ubuntu:16.04 as builder
RUN apt-get update
RUN apt-get install -y build-essential cmake software-properties-common
RUN add-apt-repository -y ppa:chrberger/libcluon
RUN apt-get update
RUN apt-get install libcluon
ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir /tmp/build
RUN cmake /opt/sources
RUN make
RUN make test
RUN cp helloworld /tmp

# Deploy.
FROM ubuntu:16.04
WORKDIR /opt/sources
COPY --from=builder /tmp/helloworld .
CMD ["/opt/sources/helloworld"]

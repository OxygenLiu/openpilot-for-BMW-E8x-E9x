CC = clang
CCFLAGS = -O3 -fPIC -DSTM32F4 -DSTM32F413xx -I.

.PHONY: all
all: libpandasafety.so

libpandasafety.so: test.o
	$(CC) -shared -o '$@' $^ -lm

test.o: test.c
	@echo "[ CC ] $@"
	$(CC) $(CCFLAGS)  -nostartfiles -MMD -c -I../../board -I../../board/inc -o '$@' '$<'

.PHONY: clean
clean:
	rm -f libpandasafety.so test.o test.d

-include test.d

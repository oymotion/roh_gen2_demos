"""SensorData / Sample 对象池。

Python 没有 C++ 意义上的静态内存分配，但可以通过对象池复用 SensorData 与 Sample
实例，避免高采样率下每秒数千次的对象创建/GC 开销。FlatBuffers 输出 bytes 也通过
预分配 bytearray slot 复用，减少堆分配。
"""

from collections import deque
from typing import Deque, List, Optional

from sensor.sensor_data import Sample, SensorData


class SensorDataPool:
    """预分配固定数量的 SensorData 与 Sample 对象，支持借用/归还。

    使用方式：
        pool = SensorDataPool(sensor_data_slots=32, samples_per_slot=256)
        sd = pool.acquire_sensor_data()
        # ... 填充数据 ...
        pool.release_sensor_data(sd)  # 用完后归还

    注意：
    - 若池中对象全部借出，会 fallback 到新建对象。
    - 回调线程持有 SensorData 期间不应归还；当前实现把对象生命周期交给 GC，
      池只覆盖“构建/反序列化”阶段的分配。后续如需严格引用计数，可扩展。
    """

    def __init__(self, sensor_data_slots: int = 32, samples_per_slot: int = 256,
                 flatbuffer_slots: int = 32, flatbuffer_size: int = 4096):
        self._sensor_data_slots = sensor_data_slots
        self._samples_per_slot = samples_per_slot
        self._flatbuffer_slots = flatbuffer_slots
        self._flatbuffer_size = flatbuffer_size

        self._sensor_data_pool: Deque[SensorData] = deque()
        self._sample_pool: Deque[Sample] = deque()
        self._flatbuffer_pool: Deque[bytearray] = deque()

        for _ in range(sensor_data_slots):
            self._sensor_data_pool.append(SensorData())

        for _ in range(samples_per_slot):
            self._sample_pool.append(Sample())

        for _ in range(flatbuffer_slots):
            self._flatbuffer_pool.append(bytearray(flatbuffer_size))

    def acquire_sensor_data(self) -> SensorData:
        """获取一个 SensorData 实例（优先从池中取）。"""
        if self._sensor_data_pool:
            sd = self._sensor_data_pool.popleft()
            sd.reset()
            return sd
        return SensorData()

    def release_sensor_data(self, sd: SensorData) -> None:
        """归还 SensorData 实例（仅归还其 sample 对象到 sample 池）。"""
        if sd is None:
            return
        # 把 channelSamples 里的 Sample 尽量回收到 sample 池
        for channel in sd.channelSamples:
            for sample in channel:
                if len(self._sample_pool) < self._samples_per_slot:
                    sample.reset()
                    self._sample_pool.append(sample)
        sd.channelSamples.clear()
        sd.reset()
        if len(self._sensor_data_pool) < self._sensor_data_slots:
            self._sensor_data_pool.append(sd)

    def acquire_sample(self) -> Sample:
        """获取一个 Sample 实例。"""
        if self._sample_pool:
            s = self._sample_pool.popleft()
            s.reset()
            return s
        return Sample()

    def acquire_samples(self, count: int) -> List[Sample]:
        """一次性获取 count 个 Sample 实例。"""
        samples: List[Sample] = []
        for _ in range(count):
            samples.append(self.acquire_sample())
        return samples

    def release_samples(self, samples: List[Sample]) -> None:
        """归还一组 Sample 实例。"""
        for sample in samples:
            if sample is None:
                continue
            if len(self._sample_pool) < self._samples_per_slot:
                sample.reset()
                self._sample_pool.append(sample)

    def acquire_flatbuffer(self) -> bytearray:
        """获取一个预分配的 FlatBuffers 输出槽（bytearray）。"""
        if self._flatbuffer_pool:
            buf = self._flatbuffer_pool.popleft()
            buf[:] = b""
            return buf
        return bytearray(self._flatbuffer_size)

    def release_flatbuffer(self, buf: bytearray) -> None:
        """归还 FlatBuffers 输出槽。"""
        if buf is None:
            return
        if len(self._flatbuffer_pool) < self._flatbuffer_slots:
            buf[:] = b""
            self._flatbuffer_pool.append(buf)

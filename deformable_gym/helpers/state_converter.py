import numpy as np
from pytransform3d import transformations as pt


class VectorConverter:
    """Convert between different vector representations.

    Example
    -------

    .. code-block:: python

        vc = VectorConverter(["x", "y", "z"], ["z", "z", ("a", 0.0), "x", "y"])
        result = vc(np.array([1, 2, 3]))

    Parameters
    ----------
    input_fields : list of str
        Names of fields of the input vector.

    output_fields : list of str, tuple, or list
        Names of fields of the output vector. You can add a constant field
        with the name 'a' by including ('a', value) or ['a', value] in the
        list of output fields.
    """
    def __init__(self, input_fields, output_fields):
        additional_keys = []
        additional_values = []
        output_keys = []
        for of in output_fields:
            if isinstance(of, tuple) or isinstance(of, list):
                key, value = of
                additional_keys.append(key)
                additional_values.append(value)
                output_keys.append(key)
            else:
                assert isinstance(of, str)
                output_keys.append(of)
        self.all_keys = input_fields + additional_keys
        self.additional_values = np.array(additional_values, dtype=float)
        self.indices = np.array(
            [self.all_keys.index(key) for key in output_keys], dtype=int)

    def __call__(self, vector):
        """Convert input fields to output fields.

        Parameters
        ----------
        vector : array-like, shape (len(input_fields),)
            Input vector.

        Returns
        -------
        result : array, shape (len(output_fields),)
            Output vector.
        """
        modified_state = np.hstack((vector, self.additional_values))
        return modified_state[self.indices]


class ActionCoupling:
    def __init__(self, action):
        self.action = action

    def coupling(self, y, yd):
        return np.zeros_like(self.action), self.action


class TransformedDMP:
    def __init__(self, dmp, dmp_base2world):
        self.dmp = dmp
        self.dmp_base2world = dmp_base2world
        self.world2dmp_base = pt.invert_transform(dmp_base2world)
        self.last_yd = None

    def step(self, last_y, coupling_term=None):
        if self.last_yd is None:
            self.last_yd = np.zeros_like(last_y)
        last_y = np.copy(last_y)

        last_y[:7] = self._world2base_link(last_y[:7])
        y, self.last_yd[:] = self.dmp.step(
            last_y, self.last_yd, coupling_term=coupling_term)
        y[:7] = self._base_link2world(y[:7])
        return y

    def _world2base_link(self, pose):
        ee2world = pt.transform_from_pq(pose)
        ee2dmp_base = pt.concat(ee2world, self.world2dmp_base)
        return pt.pq_from_transform(ee2dmp_base, self.last_yd)

    def _base_link2world(self, pose):
        ee2dmp_base = pt.transform_from_pq(pose)
        ee2world = pt.concat(ee2dmp_base, self.dmp_base2world)
        return pt.pq_from_transform(ee2world)

    @property
    def start_y(self):
        start_y = np.copy(self.dmp.start_y)
        start_y[:7] = self._base_link2world(start_y[:7])
        return start_y

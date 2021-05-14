/*!
 * Copyright (c) Trevor Richard
 * See LICENSE for more details
 */

interface VectorLike {
    x: number
    y: number
    z?: number
}

interface QuaternionLike {
    a: number
    i: number
    j: number
    k: number
}

class Vector implements VectorLike {
    x: number
    y: number
    z: number

    constructor()
    constructor(v: VectorLike)
    constructor(x: number, y: number)
    constructor(x: number, y: number, z: number)
    constructor(v?: VectorLike | number, y?: number, z?: number) {
        if (v === undefined) {
            this.x = 0;
            this.y = 0;
            this.z = 0;
        } else {
            if (typeof v === "number") {
                if (y === undefined) {
                    throw new Error("Missing y value for Vector constructor.");
                }
                this.x = v;
                this.y = y;
                this.z = z || 0;
            } else {
                this.x = v.x;
                this.y = v.y;
                this.z = v.z || 0;
            }
        }
    }

    add(v: Vector) {
        return new Vector({
            x: this.x + v.x,
            y: this.y + v.y,
            z: this.z + v.z
        });
    }

    neg() {
        return new Vector({
            x: -this.x,
            y: -this.y,
            z: -this.z
        });
    }

    sub(v: Vector) {
        return this.add(v.neg());
    }

    mul(a: number | Vector) {
        if (typeof a === "number") {
            a = new Vector(a, a, a);
        }
        return new Vector({
            x: this.x * a.x,
            y: this.y * a.y,
            z: this.z * a.z
        });
    }

    mag() {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    norm() {
        const mag = this.mag();
        if (mag == 0) return new Vector({ x: 0, y: 0, z: 0 });
        return new Vector({
            x: this.x / mag,
            y: this.y / mag,
            z: this.z / mag
        });
    }

    cross(v: Vector) {
        return new Vector(
            this.y * v.z - this.z * v.y,
            -(this.x * v.z - this.z * v.x),
            this.x * v.y - this.y * v.x
        );
    }

    dot(v: Vector) {
        return (this.x * v.x + this.y * v.y + this.z * v.z);
    }

    q_rotate(q: Quaternion) {
        return this.quaternion_rotate(q);
    }

    quaternion_rotate(q: Quaternion) {
        q = q.normalize();
        let p = new Quaternion(0, this.x, this.y, this.z);
        let c = q.conjugate();
        p = q.q_mul(p).q_mul(c);// two quaternion multiplications with this vector as a quaternion with real part '0'
        return new Vector(p.i, p.j, p.k);
    }

    transform(M: Array<Array<number>>) {
        let x = this.x * M[0][0] + this.y * M[0][1] + this.z * M[0][2] + (M[0][3] || 0);
        let y = this.x * M[1][0] + this.y * M[1][1] + this.z * M[1][2] + (M[1][3] || 0);
        let z = this.x * M[2][0] + this.y * M[2][1] + this.z * M[2][2] + (M[2][3] || 0);
        return new Vector(x, y, z);
    }

    rotate_axis(axis: Vector, angle: number) {
        return this.q_rotate(Quaternion.from_axis_rotation(axis, angle));
    }

    rotate_x(xAngle: number) {
        return this.rotate_axis(new Vector(1, 0, 0), xAngle);
    }

    rotate_y(yAngle: number) {
        return this.rotate_axis(new Vector(0, 1, 0), yAngle);
    }

    rotate_z(zAngle: number) {
        return this.rotate_axis(new Vector(0, 0, 1), zAngle);
    }

    angle_between(v: Vector) {
        let denominator = (this.mag() * v.mag());
        if (denominator == 0) {
            return 0;
        }
        return Math.acos(Math.min(Math.max(this.dot(v) / denominator, -1), 1));
    }

    project(v: Vector) {
        const theta = this.angle_between(v);
        return v.norm().mul(Math.cos(theta) * this.mag());
    }

    planar_project(origin: Vector, normal: Vector) {
        const diff = origin.sub(this);
        return this.add(diff.project(normal));
    }

    project_2d(origin: Vector, normal: Vector, y_axis: Vector) {
        const proj = this.planar_project(origin, normal).sub(origin);
        const theta = proj.angle_between(y_axis)
        const theta2 = proj.angle_between(y_axis.rotate_axis(normal, Math.PI / 2));
        const mag = proj.mag();
        return new Vector(Math.cos(theta2) * mag, Math.cos(theta) * mag, 0);
    }
}

class Quaternion implements QuaternionLike {

    public a: number
    public i: number
    public j: number
    public k: number

    constructor()
    constructor(q: QuaternionLike)
    constructor(a: number, i: number, j: number, k: number)
    constructor(qa?: QuaternionLike | number, i?: number, j?: number, k?: number) {
        if (qa === undefined) {
            this.a = 1;
            this.i = 0;
            this.j = 0;
            this.k = 0;
        } else {
            if (typeof qa === "number") {
                if (i === undefined || j === undefined || k === undefined) throw new Error("Missing i, j, or k values in Quaternion constructor...");
                this.a = qa;
                this.i = i;
                this.j = j;
                this.k = k;
            } else {
                this.a = qa.a;
                this.i = qa.i;
                this.j = qa.j;
                this.k = qa.k;
            }
        }
    }

    add(q: Quaternion) {
        return new Quaternion(this.a + q.a, this.i + q.i, this.j + q.j, this.k + q.k);
    }

    sub(q: Quaternion) {
        return new Quaternion(this.a - q.a, this.i - q.i, this.j - q.j, this.k - q.k);
    }

    conjugate() {
        return new Quaternion(this.a, -this.i, -this.j, -this.k);
    }

    mul(a: number) {
        return new Quaternion(this.a * a, this.i * a, this.j * a, this.k * a);
    }

    div(a: number) {
        if (a == 0) {
            throw new Error("Cannot divide by 0.");
        }
        return new Quaternion(this.a / a, this.i / a, this.j / a, this.k / a);
    }

    mag() {
        return Math.sqrt(this.a * this.a + this.i * this.i + this.j * this.j + this.k * this.k);
    }

    normalize() {
        return this.div(this.mag());
    }

    q_rotate(q: Quaternion) {
        return this.quaternion_rotate(q);
    }

    quaternion_rotate(q: Quaternion) {
        return q.normalize().quaternion_multiply(this.normalize());
    }

    q_mul(q: Quaternion) {
        return this.quaternion_multiply(q);
    }

    quaternion_multiply(q: Quaternion) {
        return new Quaternion(
            this.a * q.a - this.i * q.i - this.j * q.j - this.k * q.k,
            this.a * q.i + this.i * q.a + this.j * q.k - this.k * q.j,
            this.a * q.j - this.i * q.k + this.j * q.a + this.k * q.i,
            this.a * q.k + this.i * q.j - this.j * q.i + this.k * q.a
        );
    }

    rotate_axis(axis: Vector, angle: number) {
        let q = Quaternion.from_axis_rotation(axis, angle);
        return this.quaternion_rotate(q);
    }

    rotate_x(a: number) {
        return this.rotate_axis(new Vector(1, 0, 0), a);
    }

    rotate_y(a: number) {
        return this.rotate_axis(new Vector(0, 1, 0), a);
    }

    rotate_z(a: number) {
        return this.rotate_axis(new Vector(0, 0, 1), a);
    }

    get_rotation_matrix() {
        let a = this.a;
        let i = this.i;
        let j = this.j;
        let k = this.k;
        let M = [
            [
                (a * a + i * i - j * j - k * k),
                (2 * (i * j - a * k)),
                (2 * (a * j + i * k))
            ],
            [
                (2 * (i * j + a * k)),
                (a * a - i * i + j * j - k * k),
                (2 * (-a * i + j * k))
            ],
            [
                (2 * (-a * j + i * k)),
                (2 * (a * i + j * k)),
                (a * a - i * i - j * j + k * k)
            ]
        ];
        return M;
    }

    copy() {
        return new Quaternion(this.a, this.i, this.j, this.k);
    }

    static from_axis_rotation(axis: Vector, angle: number) {
        let a = Math.cos(angle / 2.0);
        let s = Math.sin(angle / 2.0);
        axis = axis.norm();
        var i = axis.x * s;
        var j = axis.y * s;
        var k = axis.z * s;
        return (new Quaternion(a, i, j, k)).normalize();
    }
}
const TAR_BLOCK_BYTES = 512;
const textDecoder = new TextDecoder();

function readString(bytes, offset, length) {
    const field = bytes.subarray(offset, offset + length);
    const end = field.indexOf(0);
    return textDecoder.decode(end >= 0 ? field.subarray(0, end) : field).trim();
}

function readOctal(bytes, offset, length) {
    const value = readString(bytes, offset, length).replace(/\0/g, '').trim();
    return value ? Number.parseInt(value, 8) : 0;
}

function isEmptyBlock(bytes, offset) {
    for (let index = offset; index < offset + TAR_BLOCK_BYTES; index += 1) {
        if (bytes[index] !== 0) return false;
    }
    return true;
}

export function normalizeTarPath(path) {
    return path.replace(/^\.\//, '').replace(/^\/+/, '');
}

export function parseTarArchive(input) {
    const bytes = input instanceof Uint8Array ? input : new Uint8Array(input);
    const files = new Map();
    let offset = 0;

    while (offset + TAR_BLOCK_BYTES <= bytes.length) {
        if (isEmptyBlock(bytes, offset)) break;

        const name = readString(bytes, offset, 100);
        const prefix = readString(bytes, offset + 345, 155);
        const size = readOctal(bytes, offset + 124, 12);
        const type = readString(bytes, offset + 156, 1);
        const combinedName = normalizeTarPath(prefix ? `${prefix}/${name}` : name);
        const dataStart = offset + TAR_BLOCK_BYTES;
        const dataEnd = dataStart + size;
        if (dataEnd > bytes.length) {
            throw new Error(`Truncated tar entry: ${combinedName}`);
        }

        if (combinedName && (type === '' || type === '0')) {
            files.set(combinedName, bytes.slice(dataStart, dataEnd));
        }
        offset = dataStart + Math.ceil(size / TAR_BLOCK_BYTES) * TAR_BLOCK_BYTES;
    }

    return files;
}

export async function fetchTarGz(url) {
    if (typeof DecompressionStream === 'undefined') {
        throw new Error('This browser does not support gzip decompression.');
    }
    const response = await fetch(url);
    if (!response.ok) {
        throw new Error(`Could not load the G1 model bundle (${response.status}).`);
    }
    if (!response.body) {
        throw new Error('The G1 model response did not contain a readable body.');
    }
    const decompressed = response.body.pipeThrough(new DecompressionStream('gzip'));
    return parseTarArchive(await new Response(decompressed).arrayBuffer());
}

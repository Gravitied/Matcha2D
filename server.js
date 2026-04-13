import http from 'node:http'
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const __filename = fileURLToPath(import.meta.url)
const __dirname = path.dirname(__filename)
const root = path.resolve(__dirname)

// Serves files as-is. Demo HTML uses bare imports (@matcha2d/*, @dimforge/*); use `npm run dev`
// (Vite) for demos that resolve those to package source and WASM.

const MIME_TYPES = {
  '.html': 'text/html',
  '.js': 'application/javascript',
  '.mjs': 'application/javascript',
  '.css': 'text/css',
  '.json': 'application/json',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.gif': 'image/gif',
  '.svg': 'image/svg+xml',
  '.ico': 'image/x-icon',
  '.wasm': 'application/wasm',
  '.map': 'application/json',
}

const server = http.createServer((req, res) => {
  let urlPath = req.url.split('?')[0]
  if (urlPath === '/' || urlPath === '/demo/') {
    urlPath = '/demo/demo-hub.html'
  }

  const filePath = path.join(root, urlPath)
  const ext = path.extname(filePath)
  const mimeType = MIME_TYPES[ext] || 'application/octet-stream'

  fs.readFile(filePath, (err, data) => {
    if (err) {
      res.writeHead(404)
      res.end('Not found')
      return
    }
    res.writeHead(200, { 'Content-Type': mimeType })
    res.end(data)
  })
})

const PORT = 3080
server.listen(PORT, () => {
  console.log(`Static file server at http://localhost:${PORT} (no bare-import resolution).`)
  console.log(`Demos with live packages: npm run dev → http://localhost:3000`)
})
